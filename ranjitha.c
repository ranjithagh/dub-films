1.
curl -s -X POST -H 'Content-Type: application/json' http://localhost:8083/connectors -d '{
    "name" : "mqtt-source",
"config" : {
    "connector.class" : "io.confluent.connect.mqtt.MqttSourceConnector",
    "tasks.max" : "10",
    "mqtt.server.uri" : "tcp://127.0.0.1:1883",
    "mqtt.topics" : "temperature1",
    "kafka.topic" : "temperature1",
    "key.converter": "org.apache.kafka.connect.storage.StringConverter",
    "value.converter": "org.apache.kafka.connect.converters.ByteArrayConverter",
    "confluent.topic.bootstrap.servers": "localhost:9092",
    "confluent.topic.replication.factor": "1",
    "confluent.license":""
}
}'

2.
bin/kafka-console-consumer --bootstrap-server localhost:9092 - --property print.key=true --property key.separator=" : " --topic temperature1

3.ksql> CREATE STREAM TEMPERATURE1(DATA VARCHAR) WITH (KAFKA_TOPIC ='temperature1', VALUE_FORMAT ='DELIMITED');
4.ksql> DESCRIBE STRAM TEMPERATURE1;
5.ksql> SET 'auto.offset.reset' = 'earliest';
6.ksql> SELECT DATA FROM TEMPERATURE1 EMIT CHANGES;
7.ksql>  CREATE STREAM TEMP WITH (VALUE_FORMAT='AVRO') AS SELECT * FROM TEMPERATURE1;


8.bin/kafka-avro-console-consumer --bootstrap-server localhost:9092 --property schema.registry.url=http://localhost:8081 --topic TEMP --from-beginning | jq '.'

9.curl -X POST http://localhost:8083/connectors -H "Content-Type: application/json" -d '{
"name": "mysql1",
"config" : {
  "key.converter.schema.registry.url": "http://localhost:8081",
  "value.converter.schema.registry.url": "http://localhost:8081",
  "connector.class": "io.confluent.connect.jdbc.JdbcSinkConnector",
  "key.converter": "org.apache.kafka.connect.storage.StringConverter",
  "value.converter": "io.confluent.connect.avro.AvroConverter",
  "key.converter.schemas.enable":"false",
  "value.converter.schemas.enable":"true",
  "config.action.reload": "restart",
  "errors.log.enable": "true",
  "errors.log.include.messages": "true",
  "print.key": "true",
  "errors.tolerance": "all",
  "topics": "TEMP",
  "connection.url": "jdbc:mysql://localhost:3306/kafka",
  "connection.user": "maniramesh",
  "connection.password": "mbmr",
  "auto.create": "true"
}
}'

10.
 bin/confluent local status mysql1

 11.in mysql if v check for table under our database table will be created and data will b getting stored
 
https://www.confluent.io/blog/kafka-connect-deep-dive-converters-serialization-explained/ 
imp link
