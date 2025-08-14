# Autoware v1.0 Fixed Build Environment

This project provides a stable and reproducible build environment for **Autoware v1.0**. It includes a Docker-based setup that applies necessary patches and fixes to the Autoware source code, addressing several known build issues in the `release/v1.0` branch.

The goal is to provide a straightforward way to get a working build of this specific Autoware version, especially for users who require GPU acceleration for perception tasks.

## Prerequisites

Before you begin, ensure you have the following software installed on your system:

*   **Docker:** The containerization platform.
*   **Docker Compose:** For orchestrating the Docker container.
*   **NVIDIA GPU Drivers:** Required for running Autoware with GPU support.
*   **NVIDIA Container Toolkit:** To enable GPU access within Docker containers.

## Recommended Usage (Docker)

This is the recommended method for building and running Autoware, as it provides a clean, isolated, and reproducible environment.

### 1. Build the Docker Image

Open a terminal in the project's root directory and run the following command:

```bash
docker-compose up --build
```

This command will:
1.  Build the Docker image using the provided `Dockerfile`. This process will download the Autoware v1.0 source code, apply patches, and compile the entire workspace. This may take a significant amount of time.
2.  Start a container named `autoware` using the newly built image.

### 2. Access the Container

Once the container is running, you can open a new terminal and get a bash shell inside it:

```bash
docker exec -it autoware bash
```

### 3. Explore the Autoware Workspace

The Autoware workspace is located at `/workspace/autoware` inside the container. You can now work with the pre-built Autoware software.

For example, to source the ROS 2 environment:
```bash
source /workspace/autoware/install/setup.bash
```

## Alternative Usage (Direct Host Build)

For advanced users who prefer to build Autoware directly on their host machine, a standalone build script is provided.

**Disclaimer:** This script will install packages and create files in your home directory (`$HOME/workspace`). It is recommended to use the Docker approach to avoid polluting your host system.

To run the script:
```bash
./build_autoware.sh
```

## Fixes and Patches

This project applies several fixes to the standard Autoware v1.0 release to ensure a successful build. These include:

*   **Git Commit Revert:** Reverts commit `896fd14` to fix an issue with repository definitions in `autoware.repos`.
*   **TensorRT Dependencies:** Adds missing `libnvinfer-headers-dev` packages required for TensorRT integration.
*   **Nebula Driver Path:** Corrects an invalid include path in the `nebula_decoders` package.
*   **Multi-Object Tracker Dependencies:** Adds a missing `diagnostic_updater` dependency to the `multi_object_tracker` package.
*   **Grid Map Libraries:** Explicitly installs and links `grid_map` libraries to resolve build failures.
