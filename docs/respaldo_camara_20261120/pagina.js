
  let sonido = false, audio = null, ultimoBeep = 0;
  const boton = document.getElementById('sonido');
  boton.onclick = () => {
    sonido = !sonido;
    if (sonido && !audio) audio = new (window.AudioContext || window.webkitAudioContext)();
    boton.textContent = sonido ? '🔊 Sonido activado' : '🔇 Activar sonido de alerta';
    boton.classList.toggle('on', sonido);
  };
  function beep() {
    if (!sonido || !audio) return;
    const o = audio.createOscillator(), g = audio.createGain();
    o.frequency.value = 880; o.connect(g); g.connect(audio.destination);
    g.gain.setValueAtTime(0.3, audio.currentTime);
    g.gain.exponentialRampToValueAtTime(0.001, audio.currentTime + 0.4);
    o.start(); o.stop(audio.currentTime + 0.4);
  }
  let historialPrevio = '';
  function mostrar(e) {
      document.body.classList.remove('sinconexion');
      document.body.classList.toggle('alerta', e.alerta);
      document.getElementById('aviso').textContent = e.alerta ? '⚠ ¡GRIETA DETECTADA!' : 'Sin grietas';
      document.title = e.alerta ? '⚠ GRIETA · Detector' : 'Detector de grietas';
      document.getElementById('grietas').textContent = e.grietas;
      document.getElementById('conf').textContent = e.confianza_max.toFixed(2);
      document.getElementById('dps').textContent = e.detecciones_por_seg.toFixed(1);
      document.getElementById('total').textContent = e.total_alertas;
      if (e.alerta && Date.now() - ultimoBeep > 1500) { beep(); ultimoBeep = Date.now(); }
      mostrarImu(e.imu);
      mostrarBaterias(e.baterias);
      // Las fotos del historial solo se vuelven a pedir si cambió la lista
      const clave = e.historial.map(h => h.foto).join(',');
      if (clave !== historialPrevio) {
        historialPrevio = clave;
        document.getElementById('historial').innerHTML = e.historial.length ? e.historial.slice(0, 8).map(h =>
          `<li><a href="/capturas/${h.foto}" target="_blank"><img src="/capturas/${h.foto}" alt="" loading="lazy">
           <div><b>${h.hora}</b><br>${h.grietas} grieta(s) · conf. ${h.confianza.toFixed(2)}</div></a></li>`
        ).join('') : '<li class="vacio">Sin detecciones todavía</li>';
      }
  }
  function mostrarBaterias(baterias) {
    for (const clave of ['control', 'traccion', 'edf']) {
      const bateria = baterias?.[clave];
      const nivel = bateria?.porcentaje == null ? '– %' : bateria.porcentaje + ' %';
      document.getElementById('bat-' + clave).textContent = bateria
        ? bateria.voltios.toLocaleString('es', { minimumFractionDigits: 2, maximumFractionDigits: 2 }) + ' V · ' + nivel
        : 'Sin datos';
      document.getElementById('bat-' + clave + '-aviso').textContent = bateria?.aviso || '';
    }
  }
  function mostrarImu(m) {
    const fija = (id, txt) => document.getElementById(id).innerHTML = txt;
    const grados = v => v === null || v === undefined ? '–' : v.toFixed(1) + '<small>°</small>';
    if (!m) {
      ['roll', 'pitch', 'yaw', 'vfrec', 'vint', 'vamp', 'veje'].forEach(id => fija(id, '–'));
      fija('imu-estado', 'Sin datos');
      fija('imu-nota', 'Inicia el emisor del BNO085 (start_gecko_pc.sh en el PC).');
      return;
    }
    fija('roll', grados(m.roll)); fija('pitch', grados(m.pitch)); fija('yaw', grados(m.yaw));
    if (m.calibrando) {
      ['vfrec', 'vint', 'vamp', 'veje'].forEach(id => fija(id, '–'));
      fija('imu-estado', 'Midiendo…');
      fija('imu-nota', 'Reuniendo muestras del giroscopio…');
      return;
    }
    fija('imu-estado', 'En vivo');
    fija('vfrec', m.frecuencia === null ? 'Sin vibración' : m.frecuencia.toFixed(1) + ' <small>Hz</small>');
    fija('vint', m.intensidad.toFixed(1) + ' <small>°/s</small>');
    fija('vamp', m.amplitud === null ? '–' : '±' + m.amplitud.toFixed(2) + '<small>°</small>');
    fija('veje', m.eje);
    fija('imu-nota', `Giroscopio a ${m.muestreo} Hz · mide vibraciones hasta ${m.max_medible} Hz`);
  }
  function sinConexion() {
    document.body.classList.add('sinconexion');
    document.body.classList.remove('alerta');
    document.getElementById('aviso').textContent = 'Sin conexión con la Raspberry';
    mostrarBaterias(null);
  }

  // --- Vídeo con FPS y latencia ---
  // El vídeo se lee con fetch (no con <img>) para conocer la hora de captura de cada
  // imagen (cabecera X-Captura, reloj de la Raspberry). El reloj del navegador se
  // sincroniza con /reloj usando la petición de menor ida y vuelta.
  const cam = document.getElementById('camara'), ctx = cam.getContext('2d');
  const META_MS = 150;
  let desfase = null, rttWifi = null;      // desfase = reloj Raspberry - reloj navegador
  let dibujos = [], latencias = [];
  let pendiente = null, dibujando = false, videoAbierto = null;

  async function sincronizar() {
    let mejor = null;
    for (let i = 0; i < 5; i++) {
      const t0 = Date.now();
      try {
        const { t } = await (await fetch('/reloj', { cache: 'no-store' })).json();
        const t1 = Date.now();
        if (!mejor || t1 - t0 < mejor.rtt) mejor = { rtt: t1 - t0, desfase: t - (t0 + t1) / 2 };
      } catch (e) { return; }
    }
    desfase = mejor.desfase; rttWifi = mejor.rtt;
  }

  function buscarFinCabecera(b, desde) {
    for (let i = desde; i + 3 < b.length; i++)
      if (b[i] === 13 && b[i + 1] === 10 && b[i + 2] === 13 && b[i + 3] === 10) return i;
    return -1;
  }

  async function leerVideo(senal) {
    const resp = await fetch('/video?t=' + Date.now(), { signal: senal, cache: 'no-store' });
    const lector = resp.body.getReader(), texto = new TextDecoder();
    let buf = new Uint8Array(0);
    for (;;) {
      const { value, done } = await lector.read();
      if (done) throw new Error('fin del vídeo');
      const unido = new Uint8Array(buf.length + value.length);
      unido.set(buf); unido.set(value, buf.length); buf = unido;
      for (;;) {
        const fin = buscarFinCabecera(buf, 0);
        if (fin < 0) break;
        const cab = texto.decode(buf.subarray(0, fin));
        const largo = parseInt((/Content-Length:\s*(\d+)/i.exec(cab) || [])[1]);
        const captura = parseFloat((/X-Captura:\s*([\d.]+)/i.exec(cab) || [])[1]);
        if (!(largo >= 0)) { buf = buf.slice(fin + 4); continue; }
        if (buf.length < fin + 4 + largo) break;
        mostrarCuadro(buf.slice(fin + 4, fin + 4 + largo), captura);
        buf = buf.slice(fin + 4 + largo);
      }
    }
  }

  // Si el navegador va más lento que la cámara, se dibuja solo la imagen más nueva.
  function mostrarCuadro(jpg, captura) {
    pendiente = { jpg, captura };
    if (!dibujando) dibujarSiguiente();
  }
  async function dibujarSiguiente() {
    dibujando = true;
    while (pendiente) {
      const { jpg, captura } = pendiente; pendiente = null;
      try {
        const bmp = await createImageBitmap(new Blob([jpg], { type: 'image/jpeg' }));
        if (cam.width !== bmp.width || cam.height !== bmp.height) {
          cam.width = bmp.width; cam.height = bmp.height;
        }
        ctx.drawImage(bmp, 0, 0); bmp.close();
        const ahora = Date.now();
        dibujos.push(ahora);
        if (desfase !== null && captura > 0) latencias.push(ahora - (captura - desfase));
      } catch (e) { /* imagen corrupta: se salta */ }
    }
    dibujando = false;
  }

  function actualizarHud() {
    const ahora = Date.now();
    dibujos = dibujos.filter(t => ahora - t <= 1000);
    latencias = latencias.slice(-30);
    document.getElementById('fps').textContent = videoAbierto ? `${dibujos.length} fps` : '– fps';
    const lat = document.getElementById('lat');
    if (latencias.length && dibujos.length) {
      const orden = [...latencias].sort((a, b) => a - b);
      const mediana = Math.max(0, Math.round(orden[orden.length >> 1]));
      lat.textContent = `latencia ${mediana} ms`;
      lat.className = mediana < META_MS ? 'ok' : mediana < 2 * META_MS ? 'medio' : 'mal';
    } else { lat.textContent = 'latencia –'; lat.className = ''; }
    document.getElementById('rtt').textContent = rttWifi === null ? 'Wi-Fi –' : `Wi-Fi ${Math.round(rttWifi)} ms`;
  }
  setInterval(actualizarHud, 500);
  setInterval(() => { if (videoAbierto) sincronizar(); }, 10000);

  async function abrirVideo() {
    while (videoAbierto) {
      const control = videoAbierto;
      try { await sincronizar(); await leerVideo(control.signal); }
      catch (e) { if (control.signal.aborted) return; }
      await new Promise(r => setTimeout(r, 1000));
    }
  }

  // Estado y vídeo por conexiones abiertas; se pausan si la pestaña no está visible
  // para no gastar datos ni peticiones de ngrok.
  let fuente = null;
  function conectar() {
    if (fuente) return;
    fuente = new EventSource('/eventos');
    fuente.onmessage = ev => mostrar(JSON.parse(ev.data));
    fuente.onerror = sinConexion;
    videoAbierto = new AbortController();
    abrirVideo();
  }
  function pausar() {
    if (fuente) { fuente.close(); fuente = null; }
    if (videoAbierto) { videoAbierto.abort(); videoAbierto = null; }
    latencias = []; dibujos = [];
  }
  document.addEventListener('visibilitychange', () => document.hidden ? pausar() : conectar());
  conectar();
