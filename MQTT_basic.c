       
///MQTT RECIVE DATA
        QJsonParseError error;
        QJsonDocument mensaje=QJsonDocument::fromJson(message.payload(),&error);

        if ((error.error==QJsonParseError::NoError)&&(mensaje.isObject())){ 
            QJsonValue entrada3=objeto_json["greenLed"]; //Obtengo la entrada greenLed. Esto lo puedo hacer porque el operador [] está sobrecargado
            QJsonValue entrada_pwm_1=objeto_json["pwm1"];

            if (entrada3.isBool()){
                checked=entrada3.toBool(); //Leo el valor de objeto (si fuese entero usaria toInt(), toDouble() si es doble....
                
                previousblockinstate=ui->pushButton_4->blockSignals(true);   //Esto es para evitar que el cambio de valor                                                                    //provoque otro envio al topic por el que he recibido
                    ui->pushButton_4->setChecked(checked);
                ui->pushButton_4->blockSignals(previousblockinstate);
            }

            if (entrada_pwm_1.isDouble()){
                previousblockinstate=ui->dial_pwm_1->blockSignals(true);
                    ui->dial_pwm_1->setValue(entrada_pwm_1.toDouble());
                ui->dial_pwm_1->blockSignals(previousblockinstate);
            }
        }

/// MQTT SEND DATA 
    QByteArray cadena;
    QJsonObject objeto_json;

    objeto_json["pwm1"]=ui->dial_pwm_1->value();
    objeto_json["pwm2"]=ui->dial_pwm_2->value();

    QJsonDocument mensaje(objeto_json);
    QMQTT::Message msg(0, ui->topic->text(), mensaje.toJson()); //Crea el mensaje MQTT contieniendo el mensaje en formato JSON

    //Publica el mensaje
    msg.setRetain(true);
    _client->publish(msg);


/// MQTT SUBSCRIBE 
    QString topic(ui->topic->text());
    _client->subscribe(topic,0); //Se suscribe al mismo topic en el que publica...


/// MESSAGES


    tiva.sendMessage(MESSAGE,NULL,0);

    MESSAGE_PARAMETER parametro;
    parametro.mode=index;
    tiva.sendMessage(MESSAGE,&parametro,sizeof(parametro));

/// MESSAGES RETAIN
    msg.setRetain(true);


/// MESSAGE RECEIVE

        case MESSAGE_BOTON_MICECATS:{
            MESSAGE_BOTON_MICECATS_PARAMETER parametro;

            if (check_and_extract_command_param(parameters, parameterSize, &parametro, sizeof(parametro))>0){
                
            }
            else
                status=PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
         
            break;
        }


/// COMMANDS
        sudo systemctl status mosquitto
        sudo systemctl start  mosquitto
        sudo systemctl stop   mosquitto

        mosquitto_sub -t topic
        mosquitto_pub -t topic -m "hola"





