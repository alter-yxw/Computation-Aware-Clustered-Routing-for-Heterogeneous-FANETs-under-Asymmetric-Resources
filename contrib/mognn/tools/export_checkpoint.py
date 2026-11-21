#!/usr/bin/env python3
"""One-time TensorFlow -> dependency-free C++ export of the actual author weights."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--checkpoint', type=Path, default=root / 'upstream/model_ChebConv_BAT800_a5_c5_ACO_agent/cp-0000.ckpt')
    parser.add_argument('--output', type=Path, default=root / 'models/BAT800.txt')
    parser.add_argument('--reader', choices=['tensorflow', 'lightweight'], default='tensorflow')
    args = parser.parse_args()
    if args.reader == 'tensorflow':
        import tensorflow as tf
        reader = tf.train.load_checkpoint(str(args.checkpoint))
        reader_version = 'tensorflow-' + tf.__version__
    else:
        from tensorflow_checkpoint_reader import py_checkpoint_reader
        reader = py_checkpoint_reader.NewCheckpointReader(str(args.checkpoint))
        reader_version = 'tensorflow-checkpoint-reader-0.2.2'
    shapes = {}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w') as out:
        out.write('MO_UPSTREAM_V1 5 1\n')
        for layer in range(5):
            prefix = f'layer_with_weights-{layer}'
            kernel = reader.get_tensor(prefix + '/kernel/.ATTRIBUTES/VARIABLE_VALUE')
            bias = reader.get_tensor(prefix + '/bias/.ATTRIBUTES/VARIABLE_VALUE')
            shapes[prefix] = list(kernel.shape)
            if kernel.shape[0] != 1:
                raise ValueError(f'Unsupported ChebConv order: {kernel.shape}')
            if kernel.dtype != np.float64 or bias.dtype != np.float64:
                raise ValueError('Released model should use float64')
            out.write(f'{kernel.shape[1]} {kernel.shape[2]}\n')
            out.write(' '.join(format(float(x), '.17g') for x in kernel.ravel()) + '\n')
            out.write(' '.join(format(float(x), '.17g') for x in bias.ravel()) + '\n')
    files = [Path(str(args.checkpoint) + s) for s in ('.index', '.data-00000-of-00001')]
    manifest = {
        'upstream': 'https://github.com/zhongyuanzhao/multihop-offload',
        'commit': '524bc4ef6af2a2a8c238b91470d10d2aeda14915',
        'reader': reader_version, 'checkpoint': args.checkpoint.name,
        'chebyshev_order': 1, 'leaky_relu_alpha': float(np.float32(.2)),
        'features': ['virtual', 'rate', 'arrival', 'server'], 'kernel_shapes': shapes,
        'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files + [args.output]},
        'paper_numerical_reproduction': False,
    }
    args.output.with_suffix('.manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
