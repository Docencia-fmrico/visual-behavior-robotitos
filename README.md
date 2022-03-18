[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7214339&assignment_repo_type=AssignmentRepo)
# visual-behaviors

## Group: ROBOTITOS

## Members:
* Oriana Acosta Joaquim (o.acosta.2019@alumnos.urjc.es)
* Juan Fernández Eva (j.fernandezev.2018@alumnos.urjc.es)
* Jaime Avilleira García (j.avilleira.2019@alumnos.urjc.es)
* Francisco Gómez López (f.gomezl.2019@alumnos.urjc.es

## Propuesta:

   La práctica se compone de tres partes:

   1. Seguimiento visual de la pelota: El robot debe seguir una pelota. El robot debe avanzar hasta estar a un metro, aproximadamente, de la pelota. A partir de ahí, seguira orientandose hacia ella, y tratando de mantener la distancia de un metro incluso si la pelota se mueve hacia el robot. Se usará:
      1.  Un filtro de color en HSV
      2.  Una estimación de la distancia al objeto filtrado, ya sea con PointCloud o con la imagen de profundidad.
      3.  Behavior Trees para programar la lógica de control.
      4.  PIDs para la orientación y la distancia.
   2. Seguimiento visual de una persona: Similar al punto anterior, pero detectando a la persona con darket_ros.
   3. Comportamiento mixto: El robot debe seguir tanto a las personas como a las pelotas que perciba con la cámara, teniendo prioridad la pelota.

## De Máquinas de Estado a Behavior Trees.
   A diferencia de la práctica anterior. En esta para controlar el flujo de comportamiento del Kobuki se han implementado *Behavior Trees*, los cuales nos permiten un control más visual de como actua el robot.
   Los árboles se pueden visualizar aquí:
   ![KOBUKI]()



## 1. [Detectball]()
