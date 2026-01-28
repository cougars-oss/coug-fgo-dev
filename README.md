# 🌊 CoUGARs FGO Development

[![ROS2 CI](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/ros2_ci.yml)
[![Docker CI](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/docker_ci.yml/badge.svg)](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/docker_ci.yml)

[**Get Started**](#get-started) | [**Contributing**](#contributing) | [**Citations**](#citations)

CoUGARs is a low-cost, configurable AUV platform designed for multi-agent autonomy research by the [Field Robotic Systems Lab (FRoSt Lab)](https://frostlab.byu.edu) at [Brigham Young University](https://byu.edu).

--

## Get Started

> **Prerequisites:** Several gigabytes of storage, 64-bit Linux or Windows, and a competent NVIDIA GPU.

- Install Docker and set up the Linux development environment.

  **Linux (Recommended):**

  - Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).

  **Windows:**

  - Install [WSL2](https://docs.microsoft.com/en-us/windows/wsl/install).

  - Install [Docker Desktop](https://docs.docker.com/desktop/) and enable the [WSL2 backend](https://docs.docker.com/desktop/windows/wsl/).

- Open a new terminal and clone the `coug_fgo_dev` repository.

  ```bash
  git clone https://github.com/snelsondurrant/coug_fgo_dev.git
  ```

- Enter the repository and run `./compose.sh up` to pull the latest image from Docker Hub and launch the `coug_dev` tmux window inside the `cougars-ct` container.

  ```bash
  cd coug_fgo_dev && ./compose.sh up
  ```

- Choose a development workflow:

  **Simulation (HoloOcean):**

  - Detatch from the tmux window using `Ctrl+b d` and build a runtime image for [HoloOcean-ROS](https://github.com/byu-holoocean/holoocean-ros/tree/main/docker). When prompted to run `./build_container.sh`, specify the branch `nelson/fgo-dev` using `./build_container.sh -b nelson/fgo-dev`.
  
  - When finished, launch the default HoloOcean scenario in the resulting `holoocean-ct` container using `./holoocean/compose.sh up`.
  
    ```bash
    cd coug_fgo_dev && ./holoocean/compose.sh up
    ```
  
  - Open a new terminal, attach to the `coug_dev` tmux window using `./compose.sh up`, build the `coug_ws` workspace, and launch the simulation stack using `./sim_launch.sh`.
  
    ```bash
    cd ~/coug_ws && colcon build --symlink-install
    cd ~/scripts && ./sim_launch.sh
    ```

  **Recorded Data (`rosbag2`):**

  - On your host machine, copy your `rosbag2` bag into the `bags` folder at the root of the repository.

  - Inside of the `coug_dev` tmux window, build the `coug_ws` workspace and launch the development stack using `./bag_launch.sh <bag_name>`. Provide the name of your bag as the script argument.
  
    ```bash
    cd ~/coug_ws && colcon build --symlink-install
    cd ~/scripts && ./bag_launch.sh <bag_name>
    ```
--

## Contributing

- **Create a Branch:** Create a new branch using the format `name/feature` (e.g., `nelson/repo-docs`).

- **Make Changes:** Develop and debug your new feature. Add good documentation.

  > All code must pass linting checks before it can be merged. We recommend using `pre-commit` for code style and formatting during development. Set it up on your host machine using `pip install pre-commit && pre-commit install`.
  >
  > If you need to add dependencies, update the `package.xml` or `Dockerfile` in your branch and test building the image locally using `docker compose -f docker/docker-compose.yaml up -d --build`. The CI will automatically build and push the new image to Docker Hub upon merge.

- **Sync Frequently:** Regularly rebase your branch against `main` (or merge `main` into your branch) to prevent conflicts.

- **Submit a PR:** Open a pull request, ensure required tests pass, and merge once approved.

--

## Citations

Please cite our relevant publications if you find this repository useful for your research:

### CoUGARs
```bibtex
@misc{durrant2025lowcostmultiagentfleetacoustic,
  title={Low-cost Multi-agent Fleet for Acoustic Cooperative Localization Research}, 
  author={Nelson Durrant and Braden Meyers and Matthew McMurray and Clayton Smith and Brighton Anderson and Tristan Hodgins and Kalliyan Velasco and Joshua G. Mangelson},
  year={2025},
  eprint={2511.08822},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.08822}, 
}
```

### HoloOcean-ROS
```bibtex
@misc{meyers2025testingevaluationunderwatervehicle,
  title={Testing and Evaluation of Underwater Vehicle Using Hardware-In-The-Loop Simulation with HoloOcean}, 
  author={Braden Meyers and Joshua G. Mangelson},
  year={2025},
  eprint={2511.07687},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.07687}, 
}
```

### HoloOcean (General Use)
```bibtex
@inproceedings{potokar2022holooceanunderwaterroboticssim,
  author={E. Potokar and S. Ashford and M. Kaess and J. Mangelson},
  title={Holo{O}cean: An Underwater Robotics Simulator},
  booktitle={Proc. IEEE Intl. Conf. on Robotics and Automation, ICRA},
  address={Philadelphia, PA, USA},
  month={May},
  year={2022}
}
```

### Sonar Simulation
```bibtex
@inproceedings{potokar2022holooceanrealisticsonarsim,
  author={E. Potokar and K. Lay and K. Norman and D. Benham and T. Neilsen and M. Kaess and J. Mangelson},
  title={Holo{O}cean: Realistic Sonar Simulation},
  booktitle={Proc. IEEE/RSJ Intl. Conf. Intelligent Robots and Systems, IROS},
  address={Kyoto, Japan},
  month={Oct},
  year={2022}
}
```

### HoloOcean 2.0 (Features)
```bibtex
@misc{romrell2025previewholoocean20,
  title={A Preview of HoloOcean 2.0}, 
  author={Blake Romrell and Abigail Austin and Braden Meyers and Ryan Anderson and Carter Noh and Joshua G. Mangelson},
  year={2025},
  eprint={2510.06160},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2510.06160}, 
}
```
