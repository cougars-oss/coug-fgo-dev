#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Rosbag autocompletions for launch scripts

_bag_tab_completion() {
    local cur=${COMP_WORDS[COMP_CWORD]}
    local prev=${COMP_WORDS[COMP_CWORD-1]}
    local script_name=${COMP_WORDS[0]}
    local bags_dir="$HOME/bags"

    if [[ "$prev" == "-r" ]]; then
        if [[ -d "$bags_dir" ]]; then
            COMPREPLY=($(compgen -W "$(ls "$bags_dir")" -- "$cur"))
        fi
        return 0
    fi

    if [[ "$script_name" == *"bag_launch.sh"* || "$script_name" == *"viz_launch.sh"* ]]; then
        if [[ -d "$bags_dir" ]]; then
            COMPREPLY=($(compgen -W "$(ls "$bags_dir")" -- "$cur"))
        fi
        return 0
    fi
}

complete -F _bag_tab_completion bag_launch.sh scripts/bag_launch.sh ./scripts/bag_launch.sh
complete -F _bag_tab_completion sim_launch.sh scripts/sim_launch.sh ./scripts/sim_launch.sh
complete -F _bag_tab_completion viz_launch.sh scripts/viz_launch.sh ./scripts/viz_launch.sh
