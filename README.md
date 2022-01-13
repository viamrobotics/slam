# SLAM

SLAM Service V1: Wrapped slam algorithms with grpc interface to allow easy integration within the viam rdk.

**Download & install:** `git clone --recurse-submodules git@github.com:viamrobotics/slam.git`

If you happened to use `git clone` only, you won't see the libraries in the `slam-libraries` folder and will need to fetch them like this:

`git submodule update --init`

* [slam-libraries](./slam-libraries) contains all slam implementations we currently support.

## Contributing
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