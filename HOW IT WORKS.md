



/////////..........Español........../////////

#Comunicacion Wifi mediante TDP
	Varios tipos de mensaje:
		*Configuracion: Se establecen parametros de configuracion->
							-Constantes del lazo de control.
							-Modo de funcionamiento.
							-Parametros varios.
							-Modo mando (APP movil o emisora RC).
							
		*Control: Se envian periodicamente las referencias de control, y se determina que no se ha perdido la conexion.
		
		*Telemteria: Datos del estado del robot/dron.

#Control
	-Control PI-D para posicion.
		*Compensacion de empuje con el angulo.
		*Control de altura.
		
	-Control PI para para ACRO
		*Compensacion de empuje con el angulo.
		*Control de altura.
		
	-Algortimos en Q15.
	-Sensado a traves del DMP de Invesense, el ESP8266 no debera calcular la posicion.

#Motores
	-Motores con escobillas.
	-Movidos mediante PWM accionado por transistor.
	-Sin diodo de proteccion, revisar si necesario.
	-Resistencia de desacctivacion.
	