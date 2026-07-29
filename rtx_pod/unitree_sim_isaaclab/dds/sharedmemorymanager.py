import json
import time
import threading
from typing import Dict, Any, Optional
from multiprocessing import shared_memory, resource_tracker


def _detach_tracker(shm) -> None:
    """Opt a segment out of THIS process's resource_tracker.

    CPython's resource_tracker unlinks every shared_memory segment a process
    *opened* when that process exits -- even if the process did not create it.
    That footgun means a consumer restarting (or crashing) deletes the sim's
    live segment out from under it (lowstate/cameras silently drop to 0). A
    process that merely *attaches* to an existing segment must unregister it so
    only the creator controls its lifetime. Best-effort; safe to call always.
    See SIM_RESILIENCE_PLAN.md Workstream C.
    """
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


class SharedMemoryManager:
    """Shared memory manager"""
    
    def __init__(self, name: str = None, size: int = 512):
        """Initialize shared memory manager
        
        Args:
            name: shared memory name, if None, create new one
            size: shared memory size (bytes)
        """
        self.size = size
        self.lock = threading.RLock()  # reentrant lock
        self._req_name = name          # requested name, for self-healing reopen
        self.shm = None
        self._open()

    def _open(self) -> None:
        """Open (attach) or create the segment. A consumer that attaches to an
        existing named segment detaches it from the resource_tracker so it never
        unlinks a segment it didn't create (see _detach_tracker)."""
        name = self._req_name
        if name:
            try:
                self.shm = shared_memory.SharedMemory(name=name)
                self.shm_name = name
                self.created = False
                _detach_tracker(self.shm)      # attach-only: don't unlink on exit
            except FileNotFoundError:
                self.shm = shared_memory.SharedMemory(create=True, size=self.size)
                self.shm_name = self.shm.name
                self.created = True
        else:
            self.shm = shared_memory.SharedMemory(create=True, size=self.size)
            self.shm_name = self.shm.name
            self.created = True

    def _reopen(self) -> bool:
        """Re-attach after the backing segment was replaced/removed (e.g. a sim
        restart, or an accidental /dev/shm wipe). Best-effort; on failure keeps
        self.shm=None so the next call retries. SIM_RESILIENCE_PLAN.md WS-C."""
        with self.lock:
            try:
                if self.shm is not None:
                    try:
                        self.shm.close()
                    except Exception:
                        pass
                self.shm = None
                self._open()
                print(f"[shm] re-attached '{self.shm_name}'")
                return True
            except Exception as e:
                print(f"[shm] reopen failed ('{self._req_name}'): {e}")
                return False
    
    def write_data(self, data: Dict[str, Any], _retry: bool = True) -> bool:
        """Write data to shared memory

        Args:
            data: data to write

        Returns:
            bool: write success or not
        """
        try:
            with self.lock:
                json_str = json.dumps(data)
                json_bytes = json_str.encode('utf-8')

                if len(json_bytes) > self.size - 8:  # reserve 8 bytes for length and timestamp
                    print(f"Warning: Data too large for shared memory ({len(json_bytes)} > {self.size - 8})")
                    return False

                # millisecond timestamp so consumers can detect a frozen (present but
                # not advancing) feed sub-second; 32-bit bitmask keeps it in range.
                timestamp = int(time.time() * 1000) & 0xFFFFFFFF
                self.shm.buf[0:4] = timestamp.to_bytes(4, 'little')
                self.shm.buf[4:8] = len(json_bytes).to_bytes(4, 'little')

                # write data
                self.shm.buf[8:8+len(json_bytes)] = json_bytes
                return True

        except Exception as e:
            # The backing segment may have been replaced/removed (sim restart or
            # an accidental /dev/shm wipe): re-attach once and retry so the writer
            # self-heals instead of silently dropping forever. WS-C.
            if _retry and self._reopen():
                return self.write_data(data, _retry=False)
            print(f"Error writing to shared memory: {e}")
            return False
    
    def read_data(self, _retry: bool = True) -> Optional[Dict[str, Any]]:
        """Read data from shared memory

        Returns:
            Dict[str, Any]: read data dictionary, return None if failed
        """
        try:
            with self.lock:
                # read timestamp and data length
                timestamp = int.from_bytes(self.shm.buf[0:4], 'little')
                data_len = int.from_bytes(self.shm.buf[4:8], 'little')

                if data_len == 0:
                    return None

                # read data
                json_bytes = bytes(self.shm.buf[8:8+data_len])
                data = json.loads(json_bytes.decode('utf-8'))
                data['_timestamp'] = timestamp  # add timestamp information (ms)
                return data

        except Exception as e:
            # Segment replaced/removed: re-attach once and retry so the reader
            # self-heals rather than returning None forever. WS-C.
            if _retry and self._reopen():
                return self.read_data(_retry=False)
            print(f"Error reading from shared memory: {e}")
            return None
    
    def get_name(self) -> str:
        """Get shared memory name"""
        return self.shm_name
    
    def cleanup(self):
        """Clean up shared memory"""
        if hasattr(self, 'shm') and self.shm:
            self.shm.close()
            if self.created:
                try:
                    self.shm.unlink()
                except:
                    pass
    
    def __del__(self):
        """Destructor"""
        self.cleanup()
