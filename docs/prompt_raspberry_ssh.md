Necesito tu ayuda para preparar esta Raspberry Pi 5 de modo que su programa de la IMU se pueda iniciar y detener desde mi PC por SSH, sin conectarle teclado ni pantalla.

## Contexto

- El proyecto se llama Gecko (antes se llamaba "mini sumo"). Es un vehículo de inspección con una IMU BNO085 conectada a esta Raspberry.
- Esta Raspberry ejecuta un programa que lee la IMU y envía el cuaternión y la aceleración lineal por UDP, en JSON, al PC `10.42.0.1`, puerto `5005`.
- El PC crea un hotspot Wi-Fi al que se conecta esta Raspberry. Antes se llamaba `MiniSumo-IMU`; ahora se llama `Gecko-IMU` y su clave es `Gecko2026!`.
- En el PC, el script `start_gecko_pc.sh` activa el hotspot y abre Gazebo y RViz. Quiero que ese script también inicie y detenga por SSH el programa de esta Raspberry.

## Parte A: recopilar información (solo lectura)

No modifiques nada en esta parte. No muestres contraseñas, claves privadas ni el contenido de los PSK del Wi-Fi.

1. **Sistema:** usuario actual, nombre del equipo (`hostname`), versión del sistema operativo, versión de Python y direcciones IP de cada interfaz.
2. **Resolución por nombre:** si `avahi-daemon` está instalado y activo, para saber si el PC puede encontrarla como `<hostname>.local`.
3. **SSH:** si el servidor SSH está instalado, habilitado y activo; el puerto; el valor de `PasswordAuthentication` y `PubkeyAuthentication`; y si existe `~/.ssh/authorized_keys`, con cuántas claves.
4. **Wi-Fi:** qué gestor de red se usa (NetworkManager o wpa_supplicant), el nombre de la interfaz inalámbrica, y las redes guardadas con su autoconexión y prioridad. Indica si `MiniSumo-IMU` y `Gecko-IMU` están guardadas.
5. **Programa de la IMU:**
   - Carpeta del proyecto, con un listado de sus archivos, y si está en un repositorio git.
   - Archivo principal y el comando exacto con que se ejecuta, incluido si usa un entorno virtual o `sudo`.
   - Cómo se inicia hoy: a mano, como servicio de systemd, con cron, `rc.local` u otro método.
   - Dónde están configurados la IP de destino, el puerto y la frecuencia de envío.
   - Qué pasa si el PC no está disponible al arrancar: si espera, reintenta o se cierra con un error.
   - Cómo se detiene, y si maneja `SIGTERM` y `Ctrl+C` de forma limpia.
   - Si escribe registros y dónde.
   - Cualquier mención a "sumo" o "MiniSumo" en el código o la configuración, con archivo y línea.
   - El contenido completo del archivo principal si tiene menos de 300 líneas. Si es más largo, solo las partes de configuración, envío UDP y arranque.
6. **Hardware:** si I2C está habilitado, si el usuario pertenece al grupo `i2c`, y el resultado de `i2cdetect -y 1`.
7. **Servicios relacionados:** cualquier unidad de systemd, tarea de cron o script de arranque que tenga que ver con la IMU o el proyecto.

## Parte B: preparación (solo con mi confirmación)

Muéstrame primero los comandos exactos que vas a ejecutar y espera a que yo los apruebe. No borres ninguna red ni configuración existente.

1. Activar el servidor SSH para que inicie con el sistema.
2. Guardar la red `Gecko-IMU` (clave `Gecko2026!`) con autoconexión activada y mayor prioridad que `MiniSumo-IMU`. Deja `MiniSumo-IMU` guardada.
3. Autorizar la clave pública de mi PC, agregándola a `~/.ssh/authorized_keys` del usuario que ejecuta el programa, con permisos `700` para `~/.ssh` y `600` para el archivo:

   ```
   ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIAyynafDK2ijZdlSB/O9Idwxy04HJqbrCwABVfJGdmvp juanmiguel.castanod@gmail.com
   ```

4. Si `avahi-daemon` no está activo, habilitarlo.
5. No cambies el programa de la IMU ni la forma en que se inicia. Eso lo haremos desde el PC.

## Formato de la respuesta

Entrégame un único informe en Markdown con una sección por cada punto de la Parte A y, si se hizo, una sección final con lo que cambió en la Parte B. Usa bloques de código para los comandos y los fragmentos de archivos. Al final incluye una línea con el comando exacto que el PC debería ejecutar por SSH para iniciar el programa, por ejemplo `ssh usuario@hostname.local 'cd ~/proyecto && python3 main.py'`.
