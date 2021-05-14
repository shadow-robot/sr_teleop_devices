# sr_teleop_devices

Contains packages relating to hardware devices used in teleoperation.

## Mocking hardware devices for unittesting

[Umockdev](https://github.com/martinpitt/umockdev) mocks Linux devices for creating integration tests for hardware related libraries and programs.

```bash
sudo apt-get update
sudo apt-get install umockdev
```

Unfortunately, for input devices, there is a [known issue](https://github.com/martinpitt/umockdev/issues/96) which prevents being able to record events. To fix this:

1. Clone [evtest](https://github.com/freedesktop-unofficial-mirror/evtest)
```bash
git clone https://github.com/freedesktop-unofficial-mirror/evtest.git
```
2. Comment out line 1163 in [evtest.c](https://github.com/freedesktop-unofficial-mirror/evtest/blob/master/evtest.c#L1163)
3. Remove and locally make and install evtest:
```bash
sudo apt remove evtest
cd evtest
./autogen.sh
make
sudo make install
```
4. Check which event the input device is:
```bash
sudo ./evtest
```
You can then use umock dev to [record and replay input devices](https://github.com/martinpitt/umockdev#record-and-replay-input-devices) such as the pedal.


## CI Statuses

Check | Status
---|---
Build|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiMEloZzVkR3BUMDlXQWQrYzRVeVp1eDBmcVloV09iTVVOU2lHQmFvemY5Ri9zdXpXZmpqd0JIYTZ6ZGJCVXV5aFgrZzNMakZCbndyQVdFcWRMajNuKy9JPSIsIml2UGFyYW1ldGVyU3BlYyI6IjZXOEpLY2Z2a2R5SkpZK0MiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=melodic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_teleop_devices_melodic-devel_install_check/)
Style|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiQkVxK3ZETjdGUFZqM3F4M0VFb0lrZzN5dXU3VktoSDRhSktRV1RKZlFSbVhFVmhUdVVKNzR6NzJOc09VVTErMy9wa1lzaEpWNWRoK0owVmRTalAzNE1VPSIsIml2UGFyYW1ldGVyU3BlYyI6Ilo5UWZkSFNQb09kdjBCVWgiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=melodic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_teleop_devices_melodic-devel_style_check/)
Code Coverage|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiTS9FVmw0UTBBbmlnQUtZYWhKN1dBckZMUmhpUDUwdU9wSUdlSTYwTHRRWENKS2lDRlpLWUkydkgwNzRLRXVqaVVEc0VWRW9zMm1VbkxiNjl2M0gzVHM0PSIsIml2UGFyYW1ldGVyU3BlYyI6Im1aU3B6emtPdXJRcnVPZVEiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=melodic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_sr_teleop_devices_melodic-devel_code_coverage/)

