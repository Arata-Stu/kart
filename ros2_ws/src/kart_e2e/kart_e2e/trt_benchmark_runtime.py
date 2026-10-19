"""Offline native TensorRT runner: FP32 I/O, CUDA event and synchronized wall timing."""
import ctypes as C
import ctypes.util
from pathlib import Path
import sys
import time

import numpy as np

from .engine_cache import validate_engine


def cuda_runtime():
    paths=[ctypes.util.find_library('cudart'),'libcudart.so',
           '/usr/local/cuda/lib64/libcudart.so','/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so']
    for root in sys.path:
        base=Path(root or '.')/'nvidia'
        for pattern in ('cuda_runtime/lib/libcudart.so*','cu*/lib/libcudart.so*'):
            paths.extend(str(p) for p in base.glob(pattern))
    for path in paths:
        if not path: continue
        try: lib=C.CDLL(path);break
        except OSError: continue
    else: raise RuntimeError('CUDA runtime library (libcudart) not found; run in the GPU kart container')
    signatures={
        'cudaSetDevice':[C.c_int], 'cudaMalloc':[C.POINTER(C.c_void_p),C.c_size_t],
        'cudaFree':[C.c_void_p], 'cudaMemcpyAsync':[C.c_void_p,C.c_void_p,C.c_size_t,C.c_int,C.c_void_p],
        'cudaStreamCreate':[C.POINTER(C.c_void_p)],'cudaStreamSynchronize':[C.c_void_p],
        'cudaStreamDestroy':[C.c_void_p], 'cudaEventCreate':[C.POINTER(C.c_void_p)],
        'cudaEventRecord':[C.c_void_p,C.c_void_p], 'cudaEventDestroy':[C.c_void_p],
        'cudaEventElapsedTime':[C.POINTER(C.c_float),C.c_void_p,C.c_void_p]}
    for name,args in signatures.items():
        function=getattr(lib,name);function.argtypes=args;function.restype=C.c_int
    return lib


class Runner:
    def __init__(self,plan,mode,input_name,input_shape):
        import tensorrt as trt
        self.cuda=cuda_runtime();self.buffers=[];self.events=[];self.stream=None
        self.context=self.engine=self.runtime=None
        try:
            self.call('cudaSetDevice',0)
            validate_engine(plan,mode,input_name,input_shape)
            self.logger=trt.Logger(trt.Logger.WARNING)
            self.runtime=trt.Runtime(self.logger)
            self.engine=self.runtime.deserialize_cuda_engine(Path(plan).read_bytes())
            if self.engine is None: raise RuntimeError('Cannot deserialize TensorRT engine')
            self.context=self.engine.create_execution_context()
            if self.context is None: raise RuntimeError('Cannot create TensorRT execution context')
            self.name=input_name;self.shape=tuple(input_shape)
            self.output=np.empty((1,1 if mode=='steer_only' else 2),dtype=np.float32)
            self.stream=C.c_void_p();self.call('cudaStreamCreate',C.byref(self.stream))
            for size,name in ((int(np.prod(input_shape))*4,input_name),(self.output.nbytes,'control')):
                pointer=C.c_void_p();self.call('cudaMalloc',C.byref(pointer),size);self.buffers.append(pointer)
                if not self.context.set_tensor_address(name,pointer.value):
                    raise RuntimeError(f'Cannot bind TensorRT tensor: {name}')
            for _ in range(2):
                event=C.c_void_p();self.call('cudaEventCreate',C.byref(event));self.events.append(event)
            inspector=self.engine.create_engine_inspector()
            self.layer_information=inspector.get_engine_information(trt.LayerInformationFormat.JSON) if inspector else '{}'
        except BaseException:
            self.close();raise

    def call(self,name,*args):
        status=getattr(self.cuda,name)(*args)
        if status: raise RuntimeError(f'{name} failed with CUDA status {status}')

    def predict(self,value):
        if value.dtype!=np.float32 or value.shape!=self.shape or not value.flags.c_contiguous:
            raise ValueError('Expected contiguous FP32 model input')
        started=time.perf_counter()
        self.call('cudaMemcpyAsync',self.buffers[0],C.c_void_p(value.ctypes.data),value.nbytes,1,self.stream)
        self.call('cudaEventRecord',self.events[0],self.stream)
        if not self.context.execute_async_v3(stream_handle=self.stream.value):
            raise RuntimeError('TensorRT execute_async_v3 failed')
        self.call('cudaEventRecord',self.events[1],self.stream)
        self.call('cudaMemcpyAsync',C.c_void_p(self.output.ctypes.data),self.buffers[1],self.output.nbytes,2,self.stream)
        self.call('cudaStreamSynchronize',self.stream)
        wall_ms=(time.perf_counter()-started)*1000
        elapsed=C.c_float();self.call('cudaEventElapsedTime',C.byref(elapsed),*self.events)
        return self.output.copy(),wall_ms,float(elapsed.value)

    def close(self):
        if self.stream is not None and self.stream.value:
            self.cuda.cudaStreamSynchronize(self.stream)
        self.context=self.engine=self.runtime=None
        for pointer in self.buffers:
            self.cuda.cudaFree(pointer)
        for event in self.events:
            self.cuda.cudaEventDestroy(event)
        if self.stream is not None and self.stream.value: self.cuda.cudaStreamDestroy(self.stream)
        self.buffers=[];self.events=[];self.stream=None

    def __enter__(self): return self
    def __exit__(self,*args): self.close()
