# bump-and-go-with-fsm-los-ultramarinos
## BumpGo
En esta versión se nos ha pedido hacer un programa básico de bump and go cuya máquina de estados está formada por 3 estados:

1. Ir hacia delante y detecta un objeto.
2. Hacia atrás.
3. Gira a la izquierda.
### Launch
```
roslaunch fsm_bump_go bumpgo_Basic.launch
```

## BumpGo_Advanced
En esta versión se nos ha pedido hacer un programa bump and go que sepa por donde detecta un objeto cuando se choca con él usando su bumper. Su máquina de estados está formada por los siguientes estados:

Detecta un objeto por la derecha:
1. Va hacia delante.
2. Detecta un objeto.
3. Hacia atrás.
4. Gira a la izquierda.

Detecta un objeto por la izquierda:
1. Va hacia delante.
2. Detecta un objeto.
3. Hacia atrás.
4. Gira a la derecha.

Detecta un objeto de frente:
1. Va hacia delante.
2. Detecta un objeto.
3. Hacia atrás.
4. Gira a la izquierda.

### Launch
```
roslaunch fsm_bump_go bumpgo_Advanced.launch
```

## BumpGo_Laser
En esta versión se nos ha pedido hacer un programa bump and go usando el láser.

### Launch
```
roslaunch fsm_bump_go bumpgo_Laser.launch
```

## Roslint
```
catkin_make roslint_fsm_bump_go
```
### Launch Simulador
```
roslaunch robots sim.launch
```
