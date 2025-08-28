[Foxglove](https://foxglove.dev) is an integrated visualization and diagnosis tool for robotics.

<hr />

To learn more, visit the following resources:

[About](https://foxglove.dev/about)
&nbsp;•&nbsp;
[Documentation](https://docs.foxglove.dev/docs)
&nbsp;•&nbsp;
[Release notes](https://github.com/foxglove/studio/releases)
&nbsp;•&nbsp;
[Blog](https://foxglove.dev/blog)


## Build
#TODO
```
docker build -t RobotDisco/foxglove .
```

## Run

```
  docker run --rm -p "8080:8080" -v ./foxglove-layout.json:/foxglove/default-layout.json RobotDisco/foxglove
```