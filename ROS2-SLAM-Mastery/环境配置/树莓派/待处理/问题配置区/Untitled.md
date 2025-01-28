如何修改终端（从sh到bash）



显示当前使用的shell

```bash
echo $SHELL
```

强制使用bash

```ini
chsh -s /bin/bash
```

手动切换到bash

```ini
exec /bin/bash
```

选择默认终端

```bash
sudo update-alternatives --config x-terminal-emulator
```

