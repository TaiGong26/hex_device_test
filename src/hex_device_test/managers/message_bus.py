from queue import Queue
import threading
from typing import Callable, Any

# 发布订阅类
class MessageBus:
    def __init__(self):
        self._publisher = None        
        self._subscribers = {}
        
        self.message_queue = Queue(maxsize=20)
        
        # 
        self._data_lock = threading.Lock()
        self._condition = threading.Condition(self._data_lock)
        
        # status
        self._running = False
        

    def start(self):

        self._running = True
        self._publisher = threading.Thread(target=self._dispatch_loop,daemon=True)
        self._publisher.start()
    
    def shutdown(self):
        if self._publisher is not None:
            self._publisher.join(timeout=0.1)
            self._running = False
        self._subscribers.clear()
        self.message_queue.clear()
    
    # ========== pub/sub ==========
    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> bool:
        """订阅主题"""
        with self._data_lock:
            if topic not in self._subscribers:
                self._subscribers[topic] = []
            if callback not in self._subscribers[topic]:
                self._subscribers[topic].append(callback)
                return True
            return False  # 已订阅
    
    def unsubscribe(self, topic: str, callback: Callable[[Any], None]) -> bool:
        """取消订阅"""
        with self._data_lock:
            if topic in self._subscribers:
                if callback in self._subscribers[topic]:
                    self._subscribers[topic].remove(callback)
                    return True
        return False
    
    def publish(self, topic: str, data: any):
        try:
            with self._condition:
                self.message_queue.put(
                    {
                        'topic': topic,
                        'data': data
                    }
                )
                return True
        except Exception as e:
            return False
    
    def _dispatch(self, msg: dict):
        topic = msg['topic']
        data = msg['data']
        if topic not in self._subscribers:
            return
        for callback in self._subscribers[topic]:
                callback(data)

    def _dispatch_loop(self):
        while self._running:
            try:

                with self._condition:
                    self._condition.wait()
                    
                    msg = self.message_queue.get(timeout=0.1)
                    self._dispatch(msg)

            except Empty:
                continue
            except Exception as e:
                print(f"[MessageBus] publish thread error: {e}")

arm_message_bus = MessageBus()