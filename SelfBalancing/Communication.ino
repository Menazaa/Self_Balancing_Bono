
void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(SSID);
  
  WiFi.begin(SSID, PWD);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    // we can even make the ESP32 to sleep
  }
 
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}
 
 
void add_json_object(char *tag, float value) {
  JsonObject obj = jsonDocument.createNestedObject();
  obj["tag"] = tag;
  obj["value"] = value;
  
}

void handlePost() {
  if (server.hasArg("plain") == false) {
    //handle error here
  }

  String body = server.arg("plain");
  Serial.println(body);
  deserializeJson(jsonDocument, body);
  
  // Get position and arm positions 
  base_angle = jsonDocument["base_angle"];
  link_angle = jsonDocument["link_angle"];
  grabber_angle = jsonDocument["grabber_angle"];
  x_pos = jsonDocument["x_pos"];
  y_pos = jsonDocument["y_pos"];

// pass parameters to control
// Respond to the client
  jsonDocument.clear();
  add_json_object("base_angle", base_angle);
  add_json_object("link_angle", link_angle);
  add_json_object("grabber_angle", grabber_angle);
  add_json_object("distance_M", distance_M);
  add_json_object("distance_L", distance_L);
  add_json_object("distance_R", distance_R);

  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}
 
 
// setup API resources
void setup_routing() {
 
  server.on("/control", HTTP_POST, handlePost);
 
  // start server
  server.begin();
}

 