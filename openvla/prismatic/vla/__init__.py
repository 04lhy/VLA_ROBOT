# 注意：原 `from .materialize import get_vla_dataset_and_collator` 改为惰性导入。
#   materialize -> prismatic.vla.datasets (RLDS/tfds) 在本仓库副本中缺失，
#   eager import 会让整个 prismatic.vla 包（含 action_tokenizer / gazebo_demos）无法导入。
#   走 RLDS 老路（train.py）时仍会按需触发 ImportError，行为不受影响。


def __getattr__(name):
    if name == "get_vla_dataset_and_collator":
        from .materialize import get_vla_dataset_and_collator

        return get_vla_dataset_and_collator
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
