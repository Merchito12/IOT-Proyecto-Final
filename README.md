# IOT-Proyecto-Final

En los archivos del repositorio esta anexo el codigo con las especificaciones pedidas en la tarea.

Adicional, los datos que se estan enviando estan siendo simulados en la variable SENSOR_SIMULADO, en caso de querer hacerlo con el dispositivo fisico hay que poner esta variable en false.

Por otro lado el MQTT se realizo con el host de: broker.hivemq.com en el puerto 1883, para poder visualizar los datos que se estan enviando, hay que entrar en la pagina de https://www.hivemq.com/demos/websocket-client, y poner los siguientes datos:

* En el host se debe poner: broker.hivemq.com

* En el puerto se debe poner: 8884

Una vez conectado, para poder ver lo datos que se estan enviado, en las suscripciones se tiene que poner lo siguiente:

* Se debe poner unicamente: car/sensor

A continuación se deberia observar los datos que se estan enviando, en el caso de que la variable de SENSOR_SIMULADOS este en true se deberian ver como se estan enviado los datos simulados. En caso de que esta variable este en false, es decir, que se esta haciendo con el sensor fisico, entonces de deberia de poder visualizar los datos que se estan opteniendo del sensor.

Integrantes:

-Nicolas Urrea

-Julian Pedraza

-Brandon Merchan
