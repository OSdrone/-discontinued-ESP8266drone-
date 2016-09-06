
LOCAL struct esp_conn* esp_conn_udp_send;
LOCAL struct esp_conn* esp_conn_udp_receive;
LOCAL struct esp_conn* esp_conn_tcp;


void initAP(void){
	wifi_set_opmode(SOFTAP_MODE);
	
	struct softap_config *config = (struct softap_config *)calloc(1, sizeof(struct softap_config));
	
	wifi_softap_get_config(config); // Get soft-AP config first.

	sprintf(config->ssid, AP_SSID);

	config->authmode = AUTH_OPEN;
	config->ssid_len = 0; // its actual SSID length
	config->max_connection = 4;
	
	wifi_softap_set_config(config); // Set ESP8266 soft-AP config
	free(config);
}

void init_user_udp(void (* receive_function(void *, char *, unsigned short))){
	esp_conn_udp_send = (struct esp_conn *)calloc(1, sizeof(struct esp_conn));
	esp_conn_udp_receive = (struct esp_conn *)calloc(1, sizeof(struct esp_conn));
	
	esp_conn_udp_receive->type = ESPCONN_UDP;
	esp_conn_udp_receive->proto.udp = (esp_udp *)calloc(1, sizeof(esp_udp));
	esp_conn_udp_receive->proto.udp->local_port = UDP_PORT;
	espconn_regist_recvcb(esp_conn_udp_receive, receive_function); // Funcion de captura cuando llega un paquete udp al puerto de recepcion
	
	esp_conn_udp_send->type = ESPCONN_UDP;
	esp_conn_udp_send->proto.udp = (esp_udp *)calloc(1, sizeof(esp_udp));
	esp_conn_udp_send->proto.udp->remote_port = UDP_REMOTE_PORT;
	esp_conn_udp_send->proto.udp->remote_ip[0] = 192; // Broadcast
	esp_conn_udp_send->proto.udp->remote_ip[1] = 168;
	esp_conn_udp_send->proto.udp->remote_ip[2] = 4;
	esp_conn_udp_send->proto.udp->remote_ip[3] = 255;
	
	// Primero el que recibe, hay que asegurar el local port
	espconn_create(esp_conn_udp_receive);
	espconn_create(esp_conn_udp_send);
}

void send_udp(uint8_t* pdata, uint8_t length){
	// Puede haber problemas hay que asegurar antes de enviar
	esp_conn_udp_send->proto.udp->remote_port = UDP_REMOTE_PORT;
	esp_conn_udp_send->proto.udp->remote_ip[0] = 192; // Broadcast
	esp_conn_udp_send->proto.udp->remote_ip[1] = 168;
	esp_conn_udp_send->proto.udp->remote_ip[2] = 4;
	esp_conn_udp_send->proto.udp->remote_ip[3] = 255;
	
	espconn_sendto( esp_conn_udp_send, pdata, length );
}

///////////			TCP			////////////////
/*
void init_user_tcp(void){
	esp_conn_tcp = (struct esp_conn *)calloc(1, sizeof(struct esp_conn));
	
	esp_conn_tcp->type = ESPCONN_TCP;
    esp_conn_tcp->state = ESPCONN_NONE;
    esp_conn_tcp->proto.tcp = (esp_tcp *)calloc(1, sizeof(esp_tcp));
    esp_conn_tcp->proto.tcp->local_port = TCP_PORT;
    espconn_regist_connectcb(esp_conn_tcp, tcp_server_listen);

    sint8 ret = espconn_accept(&esp_conn);
}

void send_tcp(uint8_t* pdata, uint8_t length){
	espconn_send( esp_conn_tcp, pdata, length );
}
*/