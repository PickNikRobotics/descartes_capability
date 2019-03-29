# To Use this Template

- Copy this directory into the /src folder of your catkin workspace, giving it a new name as desired

```
cp -r ~/bitchin_unix/templates/picknik_blanket/ ~/ws_catkin/src/PACKAGE_NAME
cd ~/ws_catkin/src/PACKAGE_NAME
```

- `cd` into that directory

- Use script to help replace all temporary package-wide names. (Including file names)

```
./customize_package.sh
```

- Contribute any improvements back to the ``bitchin_unix`` version.

## Setup Travis for Private Repos

- First create a new ssh key just for Travis:

        ssh-keygen -t rsa -b 4096 -C YOUR_EMAIL@EMAIL.COM

- Add *public* key to Github:

   - Github settings -> SSH and GPG keys

- Add *private* key to Travis:

   - On the Travis build page for the repo, click More Options -> Settings

   - Scroll down to SSH Key section and add your key, you can ignore the default one that Travis adds.
