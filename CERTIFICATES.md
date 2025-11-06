**¿Que es el protocolo TLS, cual es su importancia y que es un certificado en ese contexto?**

TLS (Transport Layer Security) es el protocolo estándar para crear un canal cifrado y autenticado entre dos partes (por ej., ESP32 ↔ broker MQTT).
Es importante para confidencialidad,Integridad y autenticación.

**¿A que riesgos se está expuesto si no se usa TLS?**


Se expone a espionaje: el robo de las credenciales MQTT y datos que recojan los sensores.

**¿Que es un CA (Certificate Authority)?**

CA es una autoridad certificadora manejada por un tercero de confianza que firma certificatos.

**¿Que es una cadena de certificados y cual es la vigencia promedio de los eslabones de la cadena?**

La cadena de certificados se compone Leaf/servidor-> Intermedia-> Raiz(CA)
Vigencia: Leaf 90 días–1 año, Intermedias 3–10 años, Raíces 10–25 años.

**¿Que es un keystore y que es un certificate bundle?**

almacén de identidad propia (clave privada + certificado), típico del que se presenta (servidor o cliente en mTLS).
Certificate bundle (CA bundle): colección de CAs de confianza para validar a otros (p. ej., muchas raíces públicas).

**¿Que es la autenticación mutua en el contexto de TLS?**

Además de autenticar al servidor, el cliente (p. ej., ESP32) también presenta su certificado. El servidor/broker acepta solo clientes con cert válido emitido por una CA que confía.

**¿Cómo se habilita la validación de certificados en el ESP32?**

Con WiFiClientSecure,
void setup() {
  net.setCACertBundle(esp_crt_bundle_attach);
  mqtt.setServer("broker.tu-dominio.com", 8883);
}

**Si el sketch necesita conectarse a múltiples dominios con certificados generados por CAs distintos, ¿que alternativas hay?**

a) Bundle de CAs 

b) Cargar CA por conexión según el host 

c) Pinning (fijar cert/clave pública): muy seguro, pero obliga a actualizar firmware en cada rotación del servidor.

**¿Cómo se puede obtener el certificado para un dominio?**

Desde el navegador (icono de candado → exportar) 
o con OpenSSL en tu PC:
openssl s_client -showcerts -servername broker.tu-dominio.com \
  -connect broker.tu-dominio.com:8883 </dev/null

**¿A qué se hace referencia cuando se habla de llave publica y privada en el contexto de TLS?**

La privada se mantiene en secreto y firma/descifra material del handshake. La pública viaja en el certificado y permite verificar firmas o cifrar hacia esa identidad.

**¿Que pasará con el código cuando los certificados expiren?**

El handshake falla y no hay conexión. Si validas por CA/bundle, renovar el leaf del servidor no requiere cambiar el firmware. Con pinning del leaf, debes actualizar el firmware con el nuevo cert/huella.

**¿Que teoría matemática es el fundamento de la criptografía moderna? ¿Cuales son las posibles implicaciones de la computación cuántica para los métodos de criptografía actuales?**

Criptografía asimétrica basada en teoría de números (RSA, DH, ECC) y cifrado simétrico (AES/ChaCha20). La computación cuántica:


Shor rompería RSA/DH/ECDSA a escala suficiente.

Grover reduce seguridad simétrica (mitigable con claves más largas, p. ej., AES-256).

Por ello surgen métodos poscuánticos (PQC) y modos híbridos (clásico+PQC) en pruebas para TLS.
# PRUEBAS
**1. Modificar el código de conexión a MQTT para usar un puerto seguro. Sin hacer más cambios verificar que la conexión sigue  funcionando (¿o no?).** 

Al hacer el cambio de puerto

#define MQTT_PORT 1883 // antes
 
#define MQTT_PORT 8883 // después

Nos marca error, no conecta ya que se esta intentando hablar TLS(8883) con un cliente sin TLS (WifiCliente)

**2. Realizar el cambio para validar certificados, verificar que sin más cambios la comunicación falla (sin cargar el certificado al ESP32).**

Se cambio a wifiClientSecure usando 
 **#include <WiFiClientSecure.h>**

Falla la validacion debido a que no se le ha dado algun CA al ESP32 entonces esta correcto con que salga error.

**3. Agregar los certificados al código y verificar que la comunicación vuelve a funcionar (¿o no?)**

Se agregó certificados usando el bundle de CAs del esp32

#include <esp_crt_bundle.h>

 espClient.setCACertBundle(esp_crt_bundle_attach);
 
client.setServer(MQTT_SERVER, MQTT_PORT);

