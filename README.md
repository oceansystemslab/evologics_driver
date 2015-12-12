evologics_driver
================

This package holds a ROS driver node for the Evologics modems used by the OSL.

Guidelines
----------

Before attempting any modification to this repo please make sure that: 
  - you are working on an up-to-date version of the `master` branch
  - the previous working version of the repo has been tagged (`git tag --list` to check the tags)
  - you have considered the option of creating a new branch for your feature (`git checkout -b <branch_name>` to create a new branch), after all this is the recommended approach!
  - you know what are you doing!
  
Initially this repo is providing packages using the rosbuild build system until the OSL vehicles are migrated to an upgraded version of the ROS. Later the rosbuild support is going to be dropped and the master branch will offer a catkinized package format. The software development follows a couple of principles that have been proven useful during the early development of this project like [Semver][semver], for semantic versioning of releases and tags, [KISS][kiss], as a general guideline to prevent the commit of _huge_ files and _beast_ modules, and, finally, [TDD][tdd], as a principle for testing your code if you want to rely on a more pragmatic approach.

Provided functionality
----------------------

### Instant Messages:
Enables the use of the short broadcast instant messages provided by the Evologics modems. The functionality is provided through the following topics:

```
/modem/im/in
/modem/im/out
/modem/im/ack
```

The client publishes data to be transmitted in "/modem/im/out", received data are published in "/modem/im/in" and acknowledgements for a transmitted message are published in "/modem/im/ack".

### Synchronous Instant Messages:
Enables the use of the short broadcast synchronous instant messages provided by the Evologics modems. The functionality is provided through the following topics:

```
/modem/ims/in
/modem/ims/out
/modem/ims/ack
```

### Burst Messages:
Enables the use of the high bitrate burst point-to-point messages provided by the Evologics modems. The functionality is provided through the following topics:

```
/modem/burst/in
/modem/burst/out
/modem/burst/ack
```

### Driver status:
Driver status is published at a rate of 1Hz in:

```
/modem/status
```

### USBL Localisation:
USBL localisation data are published in:

```
/modem/usbl
```

### Driver Interaction:
Driver interaction is enabled by using a service provided in:
 
```
/modem/switch
```

### TODO:
Define message types.

[semver]: http://semver.org/
[kiss]: http://en.wikipedia.org/wiki/KISS_principle
[tdd]: http://en.wikipedia.org/wiki/Test-driven_development
[solid]: http://en.wikipedia.org/wiki/SOLID_(object-oriented_design)
