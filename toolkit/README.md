# VOT toolkit integration

You need the newest version of the VOT toolkit (at the moment this means the dev branch). To use the PreTZel framework within the VOT toolkit you first have to compile the binaries.

Then, you have to add the this directory (`toolkit`) and subdirectories to Matlab path. You will have to ensure that these paths are always available (otherwise some functions will be missing). Once the workspace is created (e.g. for the AMP benchmark), the paths to the simulator and player executables have to be specified:

```
pretzel_build = '... path to the directory with executables ...';
set_global_variable('pretzel_simulator', fullfile(pretzel_build, 'pretzel_simulator'));
set_global_variable('pretzel_player', fullfile(pretzel_build, 'pretzel_player'));
```

## Note on tracker integration

The trackers that can be evaluated with the PreTZel framework have to support TraX protocol and should be capable of receiving frames directly from the client over standard input (either `memory` or `buffer` mode). The code for most trackers that were evaluated for the ICCV2017 paper on AMP benchmark is available online and supports these formats, more implementations will follow.

## The AMP behchmark

Create a new workspace for the `amp` stack. You can then perform experiments using the generated `run_experiments` script. The analysis part (computing scores) is currently still work-in-progress and will be added later.



