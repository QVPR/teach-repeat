# Fast and Robust Bio-inspired Teach and Repeat Navigation

## Reference
If you use code contained in this repository, please cite [our paper](https://arxiv.org/abs/2010.11326):
```
@misc{dallosto2020fast,
      title={Fast and Robust Bio-inspired Teach and Repeat Navigation}, 
      author={Dominic Dall'Osto and Tobias Fischer and Michael Milford},
      year={2020},
      eprint={2010.11326},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
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
