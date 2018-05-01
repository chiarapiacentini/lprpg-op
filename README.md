# LPRPG-OP

## Dependencies:

Flex, Bison, CLP, CoinUtils, CBC, CBLAS, CGL, CPLEX

Optional: Gurobi

## Compile:

1. Dowload repository

```
git clone https://github.com/chiarapiacentini/lprpg-op.git
```

2. Go to `lprpg-op` directory

```
cd lprpg-op
```

3. Export location of CPLEX libraries:

```
export CPLEX="/my/location/to/libcplex.a"
export ILOCPLEX="/my/location/to/libilocplex.a"
export CONCERT="/my/location/to/libconcert.a"
export CPLEX_INCLUDES="/my/location/to/cplex/include"
export CONCERT_INCLUDES="/my/location/to/concert/include"
```

4. [Export location of GUROBI:]

```
export GRUOBI="/my/location/to/gurobi"
```

5. To creat Makefiles, run build.sh file

```
./build.sh
```

this script creates a folder `build`

6. Build executable lprpg-cplex [or lprpg-both if there is also gurobi]
```
cd build
make lprpg-cplex
```
or:
```
cd build
make lprpg-both
```


## Usage:

```./build/lprpg-cplex -y domainFile problemFile```

