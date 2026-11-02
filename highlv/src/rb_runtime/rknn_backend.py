#!/usr/bin/env python3
"""RKNN backend for the recurrent Beni512 policy on RV1106 (stdlib only)."""

import ctypes as C
import json
import time
from .paths import native_directory


class Input(C.Structure):
    _fields_ = [
        ("index", C.c_uint32),
        ("buf", C.c_void_p),
        ("size", C.c_uint32),
        ("pass_through", C.c_uint8),
        ("type", C.c_int),
        ("fmt", C.c_int),
    ]


class Output(C.Structure):
    _fields_ = [
        ("want_float", C.c_uint8),
        ("is_prealloc", C.c_uint8),
        ("index", C.c_uint32),
        ("buf", C.c_void_p),
        ("size", C.c_uint32),
    ]


class IoNum(C.Structure):
    _fields_ = [("n_input", C.c_uint32), ("n_output", C.c_uint32)]


class TensorAttr(C.Structure):
    _fields_ = [
        ("index", C.c_uint32),
        ("n_dims", C.c_uint32),
        ("dims", C.c_uint32 * 16),
        ("name", C.c_char * 256),
        ("n_elems", C.c_uint32),
        ("size", C.c_uint32),
        ("fmt", C.c_int),
        ("type", C.c_int),
        ("qnt_type", C.c_int),
        ("fl", C.c_int8),
        ("zp", C.c_int32),
        ("scale", C.c_float),
        ("w_stride", C.c_uint32),
        ("size_with_stride", C.c_uint32),
        ("pass_through", C.c_uint8),
        ("h_stride", C.c_uint32),
    ]


class TensorMem(C.Structure):
    _fields_ = [
        ("virt_addr", C.c_void_p),
        ("phys_addr", C.c_uint64),
        ("fd", C.c_int32),
        ("offset", C.c_int32),
        ("size", C.c_uint32),
        ("flags", C.c_uint32),
        ("priv_data", C.c_void_p),
    ]


class TensorView(C.Structure):
    _fields_ = [
        ("memory", C.c_void_p),
        ("data", C.c_void_p),
        ("values", C.POINTER(C.c_float)),
        ("n", C.c_uint32),
        ("type", C.c_int),
        ("scale", C.c_float),
        ("zp", C.c_int),
    ]


class Model:
    def __init__(self, path, emit_io=True):
        self.lib = C.CDLL("/oem/usr/lib/librknnmrt.so")
        self.lib.rknn_init.argtypes = [
            C.POINTER(C.c_uint32),
            C.c_void_p,
            C.c_uint32,
            C.c_uint32,
            C.c_void_p,
        ]
        self.lib.rknn_query.argtypes = [C.c_uint32, C.c_int, C.c_void_p, C.c_uint32]
        self.lib.rknn_inputs_set.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(Input)]
        self.lib.rknn_run.argtypes = [C.c_uint32, C.c_void_p]
        self.lib.rknn_outputs_get.argtypes = [
            C.c_uint32,
            C.c_uint32,
            C.POINTER(Output),
            C.c_void_p,
        ]
        self.lib.rknn_outputs_release.argtypes = [
            C.c_uint32,
            C.c_uint32,
            C.POINTER(Output),
        ]
        self.lib.rknn_destroy.argtypes = [C.c_uint32]
        self.lib.rknn_create_mem.argtypes = [C.c_uint32, C.c_uint32]
        self.lib.rknn_create_mem.restype = C.POINTER(TensorMem)
        self.lib.rknn_set_io_mem.argtypes = [
            C.c_uint32,
            C.POINTER(TensorMem),
            C.POINTER(TensorAttr),
        ]
        self.lib.rknn_mem_sync.argtypes = [C.c_uint32, C.POINTER(TensorMem), C.c_int]
        self.lib.rknn_destroy_mem.argtypes = [C.c_uint32, C.POINTER(TensorMem)]
        binary = open(path, "rb").read()
        self.model_data = C.create_string_buffer(binary)
        self.context = C.c_uint32()
        start = time.perf_counter_ns()
        self.check(
            self.lib.rknn_init(
                C.byref(self.context), self.model_data, len(binary), 0, None
            ),
            "rknn_init",
        )
        self.init_ms = (time.perf_counter_ns() - start) / 1e6
        io = IoNum()
        self.check(
            self.lib.rknn_query(self.context, 0, C.byref(io), C.sizeof(io)), "query_io"
        )
        if (io.n_input, io.n_output) != (2, 2):
            raise RuntimeError(
                "Expected 2 inputs and 2 outputs, got %s/%s" % (io.n_input, io.n_output)
            )
        self.input_attrs, self.output_attrs = [], []
        self.input_mems, self.output_mems = [], []
        for command, attrs, mems in (
            (1, self.input_attrs, self.input_mems),
            (2, self.output_attrs, self.output_mems),
        ):
            for index in range(2):
                attr = TensorAttr()
                attr.index = index
                self.check(
                    self.lib.rknn_query(
                        self.context, command, C.byref(attr), C.sizeof(attr)
                    ),
                    "query_attr",
                )
                mem = self.lib.rknn_create_mem(
                    self.context, attr.size_with_stride or attr.size
                )
                if not mem:
                    raise RuntimeError("rknn_create_mem failed")
                if command == 1:
                    attr.pass_through = 1
                self.check(
                    self.lib.rknn_set_io_mem(self.context, mem, C.byref(attr)),
                    "set_io_mem",
                )
                attrs.append(attr)
                mems.append(mem)
        if emit_io:
            print(
                json.dumps(
                    {
                        "event": "io",
                        "inputs": [self.describe(a) for a in self.input_attrs],
                        "outputs": [self.describe(a) for a in self.output_attrs],
                    }
                ),
                flush=True,
            )
        hidden_inputs = [
            attr.n_elems for attr in self.input_attrs if attr.n_elems != 32
        ]
        if len(hidden_inputs) != 1 or hidden_inputs[0] not in (256, 512):
            raise RuntimeError("Expected GRU256 or GRU512 hidden input")
        self.hidden_dim = hidden_inputs[0]
        if [a.n_elems for a in self.input_attrs] != [32, 512]:
            raise RuntimeError("Expected obs32 followed by hidden512")
        if sorted(a.n_elems for a in self.output_attrs) != [6, 512]:
            raise RuntimeError("Expected action6 and hidden512 outputs")
        self.fast = C.CDLL(str(native_directory() / "lib/libtensor_math.so"))
        self.fast.hcr_pack.argtypes = [
            C.POINTER(C.c_float),
            C.c_void_p,
            C.c_uint32,
            C.c_int,
            C.c_float,
            C.c_int,
        ]
        self.fast.hcr_unpack.argtypes = [
            C.c_void_p,
            C.POINTER(C.c_float),
            C.c_uint32,
            C.c_int,
            C.c_float,
            C.c_int,
        ]
        self.output_buffers = [(C.c_float * a.n_elems)() for a in self.output_attrs]
        self.obs_buffer = (C.c_float * 32)()
        self.fast.hcr_execute.argtypes = [
            C.c_uint32,
            C.c_void_p,
            C.c_void_p,
            C.POINTER(TensorView),
            C.POINTER(TensorView),
        ]
        self.run_ptr = C.cast(self.lib.rknn_run, C.c_void_p)
        self.sync_ptr = C.cast(self.lib.rknn_mem_sync, C.c_void_p)
        self.input_views = (TensorView * 2)()
        self.output_views = (TensorView * 2)()
        for attrs, mems, views in (
            (self.input_attrs, self.input_mems, self.input_views),
            (self.output_attrs, self.output_mems, self.output_views),
        ):
            for i, (attr, mem) in enumerate(zip(attrs, mems)):
                if attr.type == 0:
                    scale, zp = 1.0, 0
                else:
                    scale, zp = self.quant_params(attr)
                views[i] = TensorView(
                    C.cast(mem, C.c_void_p),
                    mem.contents.virt_addr,
                    None,
                    attr.n_elems,
                    attr.type,
                    scale,
                    zp,
                )
        for view, buf in zip(self.output_views, self.output_buffers):
            view.values = buf
        self.input_views[0].values = self.obs_buffer
        self.closed = False

    @staticmethod
    def describe(attr):
        return {
            "name": attr.name.decode(errors="replace").rstrip("\0"),
            "shape": list(attr.dims[: attr.n_dims]),
            "type": attr.type,
            "qnt_type": attr.qnt_type,
            "fl": attr.fl,
            "scale": attr.scale,
            "zp": attr.zp,
            "size": attr.size,
            "size_with_stride": attr.size_with_stride,
        }

    @staticmethod
    def check(code, label):
        if code != 0:
            raise RuntimeError("%s failed: %d" % (label, code))

    @staticmethod
    def quant_params(attr):
        if attr.qnt_type == 2:  # asymmetric affine
            return attr.scale, attr.zp
        if attr.qnt_type == 1:  # dynamic fixed point
            return 2.0 ** (-attr.fl), 0
        raise RuntimeError("Unsupported quantization: %s" % Model.describe(attr))

    def step(self, observation, hidden):
        if len(observation) != 32 or len(hidden) != 512:
            raise ValueError("Expected obs32 and hidden512")
        start = time.perf_counter_ns()
        self.obs_buffer[:] = observation
        hidden_buffer = (
            hidden if isinstance(hidden, C.Array) else (C.c_float * 512)(*hidden)
        )
        self.input_views[1].values = hidden_buffer
        native_start = time.perf_counter_ns()
        self.check(
            self.fast.hcr_execute(
                self.context,
                self.run_ptr,
                self.sync_ptr,
                self.input_views,
                self.output_views,
            ),
            "native inference",
        )
        self.last_native_ms = (time.perf_counter_ns() - native_start) / 1e6
        elapsed_us = (time.perf_counter_ns() - start) / 1000
        lengths = [a.n_elems for a in self.output_attrs]
        return (
            list(self.output_buffers[lengths.index(6)]),
            self.output_buffers[lengths.index(512)],
            elapsed_us,
        )

    def close(self):
        if self.closed:
            return
        for mem in self.input_mems + self.output_mems:
            self.lib.rknn_destroy_mem(self.context, mem)
        self.lib.rknn_destroy(self.context)
        self.closed = True
