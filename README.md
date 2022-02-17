[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6870052&assignment_repo_type=AssignmentRepo)
# fsm_bump_go
## BUMPER

La versión que usa el bumper se puede encontrar en la rama Gira_con_el_Bumper. Esta mantiene los 3 estados originales, pero añade la variable *sentido_*, cuyo valor se actualiza a 1 o -1 dependiendo de si se activa el bumper izquierdo o derecho. Después, dentro del estado *TURNING*, multiplicamos la velocidad angular por el sentido para girar al lado opuesto al bumper activado. Además, *BACKING_TIME* se ha hecho variable en vez de constante con el fin de poder determinar distintos angulos de giro dependiendo de si la colision es frontal o lateral. 

Teniendo todo esto en cuenta, la funcion **bumperCallback** tiene distintos escenarios dependiendo del bumper que se active. Una colision frontal actualiza la variable *angulo_*, que determina un *BACKING_TIME* correspondiente a un giro de 90º, pero no cambia el sentido del giro. Una colision lateral actualiza *angulo_* para girar 45º, y *sentido_* para girar en la direccion opuesta al bumper activado.


## LASER

La versión que usa el laser se puede encontrar en la rama main y la rama laser_con_funciones. Para usar el laser hemos optado cambiar el modelo de una maquina de estados a un nodo reactivo, que avanza hasta detectar un obstaculo y gira en la direccion correspondiente hasta dejar de verlo. Para ello, hemos determinado un campo de vision modulable con un *rango*, o distancia de deteccion de obstaculos, y una *apertura* variables.

La funcion **laserCallback** filtra el vector de rangos laser para seleccionar tan solo aquellos que se correspondan a la apertura deseada. Posteriormente, si dentro de esa apertura encontramos un obstaculo dentro del rango, el robot determinara a que lado se encuentra el obstaculo y girara en el sentido contrario. Si mientras el robot gira aparece otro obstaculo, el giro continuara en el mismo sentido hasta que no se detecten mas obstaculos, en cuyo caso se vuelve a avanzar hasta encontrar otro obstaculo.
