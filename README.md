# SLAM

## (In)stability Notice
> **Warning**
> This is an experimental feature. Stability is not guaranteed. Breaking changes are likely to occur, and occur often.

## Overview
This repo contains [SLAM libraries](https://github.com/viamrobotics/slam/tree/main/slam-libraries) that are integrated within Viam's [RDK](https://github.com/viamrobotics/rdk).

## Getting started

### Download and install
```bash
git clone --recurse-submodules git@github.com:viamrobotics/slam.git
```

If you happened to use `git clone` only, you won't see the libraries in the `slam-libraries` folder and will need to fetch them:

`git submodule update --init`

### Running the SLAM libraries
[slam-libraries](./slam-libraries) contains currently supported SLAM implementations. To get started with setting up and running the SLAM libraries, follow the instructions on the [slam-libraries README](./slam-libraries/README.md).

## Development
### Linting

```bash
brew install clang-format
make format
```

### Working with submodules

#### Clone and pull

1. Clone repo: `git clone --recurse-submodules git@github.com:viamrobotics/slam.git`
2. `cd slam`
    * If you forgot `--recurse-submodules` when cloning or can't find any code in the gitmodules folders, run this: `git submodule update --init`
2. `git checkout YOUR_SLAM_BRANCH`
3. Make sure your submodules are also on the correct branch by running `git submodule update --init`

#### Commit and push
1. Commit and push changes in the submodules first.
2. Commit and push changes in the `slam` library last.

Or, alternatively:
1. Commit changes in the submodules
1. Commit changes in the main repo
1. Push all changes by running `git push --recurse-submodules=on-demand`

#### Changing branches in a submodule
When changing branches in a submodule, update `.gitmodules`, e.g., changing to a branch called `kk/fix-install`:

```bash
...
[submodule "slam-libraries/cartographer"]
        path = slam-libraries/cartographer
        url = git@github.com:kkufieta/cartographer.git
        branch=kk/fix-install
```

Commit & push the changes.

When pulling those changes, run the following:
```bash
git pull
git submodule update --init --recursive
```

## License
Copyright 2022 Viam Inc.

Apache 2.0 - See [LICENSE](https://github.com/viamrobotics/slam/blob/main/LICENSE) file
