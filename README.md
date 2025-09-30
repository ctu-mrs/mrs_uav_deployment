# mrs_uav_deployment

Please run the following command in this repository once you clone it

```bash
git config core.hooksPath .ci
```

It will run a script on every commit that checks if the ssh keys are encrypted, you can run it as

```bash
sh .ci/pre-commit
```
