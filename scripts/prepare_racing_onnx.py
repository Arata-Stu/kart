#!/usr/bin/env python3
"""Expose FP32 ROS bindings on a fixed-shape Ultralytics Detect export."""
import argparse
from pathlib import Path

import onnx
from onnx import TensorProto, helper


def prepare(source, destination, num_classes=80):
    if isinstance(num_classes, bool) or not isinstance(num_classes, int) or num_classes < 1:
        raise ValueError("num_classes must be a positive integer")
    model = onnx.load(source)
    graph = model.graph
    if len(graph.input) != 1 or len(graph.output) != 1:
        raise ValueError("Expected one input and one output")
    for value, name, shape in [(graph.input[0], "images", [1, 3, 480, 640]),
                               (graph.output[0], "output0", [1, 4 + num_classes, 6300])]:
        typ = value.type.tensor_type
        if (value.name != name or typ.elem_type not in (TensorProto.FLOAT, TensorProto.FLOAT16) or
                [d.dim_value for d in typ.shape.dim] != shape):
            raise ValueError(f"Expected float binding {name} {shape}. Check export and class count.")
    if all(v.type.tensor_type.elem_type == TensorProto.FLOAT for v in (graph.input[0], graph.output[0])):
        onnx.checker.check_model(model)
        onnx.save(model, destination)
        return
    if graph.input[0].type.tensor_type.elem_type != graph.output[0].type.tensor_type.elem_type:
        raise ValueError("Expected matching input/output binding types")
    internal = {"images": "kart_images_fp16", "output0": "kart_output_fp16"}
    for node in graph.node:
        for names in (node.input, node.output):
            for i, name in enumerate(names):
                if name in internal.values():
                    raise ValueError(f"Reserved tensor name: {name}")
                names[i] = internal.get(name, name)
    for value in graph.value_info:
        value.name = internal.get(value.name, value.name)
    graph.input[0].type.tensor_type.elem_type = TensorProto.FLOAT
    graph.output[0].type.tensor_type.elem_type = TensorProto.FLOAT
    graph.node.insert(0, helper.make_node("Cast", ["images"], [internal["images"]], to=TensorProto.FLOAT16))
    graph.node.append(helper.make_node("Cast", [internal["output0"]], ["output0"], to=TensorProto.FLOAT))
    onnx.checker.check_model(model)
    onnx.save(model, destination)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--num-classes", type=int, default=80,
                        help="Detect class count (80 for COCO; 2 for racing; 1 for legacy opponent)")
    args = parser.parse_args()
    if args.source.resolve() == args.destination.resolve():
        parser.error("Choose a separate output path")
    prepare(args.source, args.destination, args.num_classes)
