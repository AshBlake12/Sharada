
```sh
python3 --version           
docker --version           
docker run hello-world     
git --version
df -h /                   
```

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
uv --version
```

If `docker run hello-world` needs sudo:

```sh
sudo usermod -aG docker $USER
newgrp docker
```


## System packages

```sh
sudo apt update
sudo apt install -y build-essential git python3-venv python3-pip \
  librsvg2-bin pngquant klayout gtkwave \
  libtinfo6 libffi-dev pkg-config
```

## OSS CAD Suite

```sh
mkdir -p ~/eda && cd ~/eda
# Get the latest linux-x64 tarball URL from:
#   https://github.com/YosysHQ/oss-cad-suite-build/releases/latest
wget <oss-cad-suite-linux-x64-YYYYMMDD.tgz URL>
tar -xzf oss-cad-suite-linux-x64-*.tgz
```

```sh
alias oss='source ~/eda/oss-cad-suite/environment'
```

```sh
oss
yosys -V && verilator --version && sby --version
```


## Project repo

```sh
git clone -b ihp-sg13cmos5l https://github.com/TinyTapeout/tt-support-tools tt
echo "tt/" >> .gitignore
```


```sh
ls -a   
```

Set tiles in `info.yaml`:

```yaml
tiles: "6x4"
```

## LibreLane and the PDK

```sh
mkdir -p ~/ttsetup
uv venv --python 3.12 ~/ttsetup/venv
source ~/ttsetup/venv/bin/activate
uv pip install -r ~/eda/protocol-emulator-asic/tt/requirements.txt
```

Environment variables in `~/ttsetup/env.sh` so you can re-source them:

```sh
# Matches the ihp-cmos5l CI (TinyTapeout/tt-gds-action@ihp-cmos5l)
export PDK_ROOT=~/ttsetup/pdk-cmos5l
export PDK=ihp-sg13cmos5l
export LIBRELANE_TAG=3.1.0.dev3
```

```sh
source ~/ttsetup/env.sh
uv pip install librelane==$LIBRELANE_TAG
```

The CMOS5L PDK is not installed by ciel; tt-support-tools passes `--manual-pdk`. Clone it at the revision the CI pins (from `install_sg13cmos5l.sh` in tt-gds-action):

```sh
IHP_PDK_REV=2bbec755dc67ca3db0261c3d6163e15735d66710
mkdir -p $PDK_ROOT
git -C $PDK_ROOT init -q
git -C $PDK_ROOT fetch -q --depth 1 https://github.com/IHP-GmbH/IHP-Open-PDK.git $IHP_PDK_REV
git -C $PDK_ROOT checkout -q FETCH_HEAD
echo "IHP-Open-PDK $IHP_PDK_REV" > $PDK_ROOT/ihp-sg13cmos5l/SOURCES
```

When the CI action bumps either version, bump `env.sh`, this file and `.devcontainer/Dockerfile` together. `ihp-sg13g2` is a different process; the `main` branch of tt-support-tools targets it, not CMOS5L.

`librelane --version`

---

## First harden

Note the `--ihp` flag. Every `tt_tool.py` command needs it. On the `ihp-sg13cmos5l` branch of tt-support-tools it selects CMOS5L.

```sh
./tt/tt_tool.py --ihp --create-user-config
./tt/tt_tool.py --ihp --harden
```

```sh
./tt/tt_tool.py --ihp --print-warnings
```

```sh
ls runs/wokwi/final/gds/          # a .gds exists
```

Record the numbers from `runs/wokwi/final/metrics.json` in `docs/area-log.md`, with the date, commit hash and PDK.

---

## Look at it

```sh
./tt/tt_tool.py --ihp --open-in-klayout
```

If the GUI fails with `could not connect to display :0`:

```sh
xhost +local:docker
```

Worth doing once today. Seeing 24 tiles of actual layout calibrates your sense of how little space you have better than any cell-count table.

---

## RTL tests

New terminal, LibreLane venv active:

```sh
cd ~/eda/protocol-emulator-asic/test
uv pip install -r requirements.txt
make -B
```

cocotb reports passing tests and produces a `.vcd`. Open it with `gtkwave` (from the OSS CAD Suite shell).

---

## UART out of a pin

Their getting-started advice, and the right first design. Replace the template's logic with a minimal UART transmitter:

- Fixed divisor, 8N1, transmit only
- One output pin, one start button on `ui_in`
- Nothing programmable yet

Then: `make -B` in `test/`, then reharden, then record the new cell count in `docs/area-log.md`.

**End-of-day success:** two rows in the area log. You now know what a trivial design costs, which is the baseline every later estimate in the proposal gets checked against.

---

## Hardcaml 

```sh
sudo apt install -y opam
opam init --bare -y
opam switch create 5.2.0
eval $(opam env)
opam install -y hardcaml hardcaml_waveterm dune
```

write a two-line combinational circuit, emit Verilog, read it. That's the whole week-2 spike — if this takes more than an hour, that's the signal to reconsider the mixed-language plan.

