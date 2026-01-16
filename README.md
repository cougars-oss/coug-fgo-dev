# 🌊 CoUGARs FGO Development

[![ROS CI](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/ros_ci.yml/badge.svg)](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/ros_ci.yml)
[![Docker CI](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/docker_ci.yml/badge.svg)](https://github.com/snelsondurrant/coug_fgo_dev/actions/workflows/docker_ci.yml)

[**Get Started**](#get-started) | [**Contributing**](#contributing) | [**Publications**](#publications)

CoUGARS is a low-cost, configurable AUV platform designed for multi-agent autonomy research by the [Field Robotic Systems Lab (FRoSt Lab)](https://frostlab.byu.edu) at [Brigham Young University](https://byu.edu).

--

## Get Started

> **NOTE:** Per the official HoloOcean (and HoloOcean-ROS) documentation, your computer must have several gigabytes of storage available, 64-bit Linux or Windows, and a competent NVIDIA GPU.

- Install Docker and set up the Linux development environment.

  **Windows:**

  - Install WSL2 by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

  - Install Docker Desktop by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

  **Linux:**

  - Install Docker Engine by following the instructions [here](https://docs.docker.com/engine/install/ubuntu/). Make sure to follow the post-installation steps to enable Docker commands without `sudo`.

- Set up GitHub SSH keys by following the instructions [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

- Open a new terminal and clone the `coug_fgo_dev` repository.

  ```bash
  git clone git@github.com:snelsondurrant/coug_fgo_dev.git
  ```

- Enter the repository and run `./compose.sh` to pull the latest image from Docker Hub and launch the `cougars-ct` container.

  ```bash
  cd coug_fgo_dev && ./compose.sh
  ```

- Exit the container and follow the instructions [here](https://github.com/byu-holoocean/holoocean-ros/tree/main/docker) to build a runtime Docker image for `holoocean-ros`. When prompted to run `./build_container.sh`, specify the branch `nelson/fgo-dev` using `./build_container.sh -b nelson/fgo-dev`.

- When finished, launch HoloOcean in the `holoocean-ct` container using `./compose.sh`.

  ```bash
  cd coug_fgo_dev/holoocean && ./compose.sh
  ```

- Open a new terminal, enter the `cougars-ct` container using `./compose.sh`, build the `coug_ws` workspace, and launch the simulation stack using `./sim_launch.sh`.

  ```bash
  cd ~/coug_ws && colcon build --symlink-install
  cd ~/scripts && ./sim_launch.sh
  ```

--

## Contributing

- **Create a Branch:** Create a new branch using the format `name/feature` (e.g., `nelson/repo-docs`).

- **Make Changes:** Develop and debug your new feature. Add good documentation.

  > **NOTE:** If you need to add dependencies, update the `package.xml` or `Dockerfile` in your branch and test building the image locally. The CI will automatically build and push the new image to Docker Hub upon merge.

- **Sync Frequently:** Regularly rebase your branch against `main` (or merge `main` into your branch) to prevent conflicts.

- **Submit a PR:** Open a pull request, ensure required tests pass, and merge once approved.

--

## Publications

Please cite our relevant publications if you find CoUGARs useful for your research:

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
