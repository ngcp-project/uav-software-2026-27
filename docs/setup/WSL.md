# WSL2 Setup

Run PowerShell as Administrator.

```powershell
wsl --install -d Ubuntu-24.04
```

After restarting, open **Ubuntu 24.04 LTS** from the Windows Start menu.

The first launch may take a minute to finish installing.

You will be asked to create:

* A Linux username
* A Linux password

Open PowerShell and run:

```powershell
wsl -l -v
```

You should see something like:

```text
NAME              STATE           VERSION
Ubuntu-24.04      Running         2
```

Enter WSL:

```powershell
wsl
```

It should start somewhere like:

```text
/mnt/c/Users/yourname
```

Run:

```bash
cd
```

Update Ubuntu:

```bash
sudo apt update
sudo apt upgrade -y
```

Install the basics:

```bash
sudo apt install -y git curl wget build-essential python3 python3-pip python3-venv
```

Create a directory to hold your repo if you want:

```bash
mkdir projects
cd projects
```

Clone the repo:

```bash
git clone --recurse-submodules https://github.com/ngcp-project/uav-software-2026-27
```

Enter the repo:

```bash
cd uav-software-2026-27
```

Open it in VS Code:

```bash
code .
```

Boom, you're basically set up for now.
