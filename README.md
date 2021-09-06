# Fast and Robust Bio-inspired Teach and Repeat Navigation

## Multimedia Material

The multimedia material accompanying the paper can be viewed [here](https://qvpr.github.io/teach-repeat/).

## Reference

If you use code contained in this repository, please cite [our paper](https://arxiv.org/abs/2010.11326):

```bibtex
@inproceedings{dall2021fast,
      title={Fast and Robust Bio-inspired Teach and Repeat Navigation},
      author={Dall'Osto, Dominic and Fischer, Tobias and Milford, Michael},
      booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems},
      year={2021}
}
```

## How-to-use

### Nodes for running teach on Jackal

- `rosnode kill twist_mux` (optional, only required for comparison with bearnav)
- `roslaunch slam_toolbox localization.launch` (optional, only required for quantitative analysis)
- `roslaunch stroll_bearnav mapping-core-jackal.launch` (optional, only required for comparison with bearnav)
- `roslaunch stroll_bearnav mapping-gui-jackal.launch`  (optional, only required for comparison with bearnav)
- `roslaunch teach_repeat data_collection_jackal.launch`

### Nodes for running repeat on Jackal

- `roslaunch slam_toolbox localization.launch` (optional, only required for quantitative analysis)
- `roslaunch teach_repeat data_matching_jackal.launch`

### Nodes for running Bearnav repeat on Jackal

- `roslaunch slam_toolbox localization.launch`
- `roslaunch stroll_bearnav navigation-core-jackal.launch`
- `roslaunch stroll_bearnav navigation-gui-jackal.launch`

### Nodes for running teach on Miro

- `roslaunch teach_repeat data_collection_miro.launch`

### Nodes for running repeat on Miro

- `roslaunch teach_repeat data_matching_miro.launch`
