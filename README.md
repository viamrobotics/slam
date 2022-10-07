# SLAM

This repo contains various slam algorithms that are wrapped with the [SLAM service grpc interface](https://github.com/viamrobotics/rdk/tree/main/proto/api/service/slam/v1) to allow easy integration within the viam rdk.

## (In)stability Notice
> **Warning**
> This is an experimental feature. Stability is not guaranteed. Breaking changes are likely to occur, and occur often.

## Setup

### Download & install
```bash
git clone --recurse-submodules git@github.com:viamrobotics/slam.git
```

If you happened to use `git clone` only, you won't see the libraries in the `slam-libraries` folder and will need to fetch them like this:

`git submodule update --init`

* [slam-libraries](./slam-libraries) contains all slam implementations we currently support.

## Linting

```bash
brew install clang-format
make format
```

## Working with submodules
When working with submodules, it is important to follow this order:

### Clone & pull
Working with gitmodules & branches takes a few more steps than usual, which are outlined below:


1. Clone repo: `git clone --recurse-submodules git@github.com:viamrobotics/slam.git`
2. `cd slam`
    * If you forgot `--recurse-submodules` when cloning or can't find any code in the gitmodules folders, run this: `git submodule update --init`
2. `git checkout YOUR_SLAM_BRANCH`
3. Make sure your submodules are also on the correct branch by running `git submodule update --init`

Happy coding!

### Commit & push
1. Commit & push the code in the submodules first
2. Commit & push the code in the `slam` library last.

Or, alternatively:
1. Commit changes in the submodules
1. Commit changes in the main repo
1. Push all changes by running `git push --recurse-submodules=on-demand`

### Changing branches in a submodule
When changing branches in a submodule, udpate `.gitmodules`, e.g. changing to a branch called `kk/fix-install`:

```bash
...
[submodule "slam-libraries/cartographer"]
        path = slam-libraries/cartographer
        url = git@github.com:kkufieta/cartographer.git
        branch=kk/fix-install
```

Commit & push the changes.

When pulling down those changes, run the following:
```bash
git pull
git submodule update --init --recursive
```

## License
Copyright 2022 Viam Inc.

Apache 2.0 - See [LICENSE](https://github.com/viamrobotics/slam/blob/main/LICENSE) file
