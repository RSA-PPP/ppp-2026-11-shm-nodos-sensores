#!/usr/bin/env python3
"""
Verificador de la Fase 4 (trama de 512 bytes con patron 0-255). SOLO LEE: nunca escribe en la SD.

USO EN VS CODE (recomendado): abre este archivo, elige el interprete de Python y pulsa el boton
"Run Python File" (triangulo). VS Code debe estar abierto COMO ADMINISTRADOR (Windows): clic derecho
sobre el icono de VS Code > "Ejecutar como administrador". El script lista los discos, pregunta el
numero de disco y el ID del nodo, verifica y permite revisar la siguiente tarjeta sin reiniciar.
Puedes fijar los valores en el bloque CONFIGURACION de abajo para que no pregunte.

Otras formas de uso (linea de comandos):
  1) Directo desde la tarjeta (sin HxD):
       python verificar_fase4.py --listar                      (muestra los discos y su numero)
       python verificar_fase4.py --disco 2 --id 1 [--pps 1] [--guardar nodo1.bin]
     Windows: abrir CMD/PowerShell COMO ADMINISTRADOR. Linux/macOS: usar sudo y la ruta
     (/dev/sdX, /dev/mmcblk0, /dev/rdiskN). En Windows --disco acepta el numero (2 -> PhysicalDrive2).
  2) Desde un volcado .bin copiado con HxD:
       python verificar_fase4.py volcado.bin --id 1

Con la tarjeta, lee el sector de diagnostico (2047), decodifica los contadores y lee exactamente
los sectores de datos escritos (desde 2048). Comprueba por sector: cabecera AA 55 AA 55, ID,
marcador 55 AA, CRC16-CCITT-FALSE (bytes 0..507), patron 0-255 y timestamp +1 por sector.
"""
import sys, os, argparse, struct, subprocess, platform

# ============================== CONFIGURACION (VS Code) ==============================
DISCO = ""               # "" = preguntar al ejecutar. En Windows: numero del disco, ej. "2"
ID_NODO = None           # None = preguntar. 1 o 2 = ID esperado del nodo
PPS = 1.0                # pulsos por segundo del ensayo (para calcular el margen); None = no calcular
GUARDAR_VOLCADO = True   # True = guarda nodo<ID>.bin junto a este script
# =====================================================================================

SECTOR = 512
SECTOR_INICIAL = 2048
SECTOR_DIAG = SECTOR_INICIAL - 1

class ErrorDisco(Exception):
    pass

def es_administrador():
    if platform.system() == "Windows":
        try:
            import ctypes
            return bool(ctypes.windll.shell32.IsUserAnAdmin())
        except Exception:
            return False
    return hasattr(os, "geteuid") and os.geteuid() == 0

def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

CAMPOS_DIAG = [("sector_actual", 6), ("pulsos INT1", 10), ("sectores llenados", 14),
               ("sectores escritos", 18), ("overflow de bufer", 22), ("errores SD", 26),
               ("tramas validas", 30), ("errores checksum", 34), ("discrepancias pulso/trama", 38),
               ("saltos de ts en tramas", 42), ("bytes fuera de trama", 46),
               ("tramas truncadas", 50), ("errores UART", 54), ("ocupacion maxima (1..2)", 58),
               ("t escritura min (us)", 62), ("t escritura max (us)", 66),
               ("t escritura prom (us)", 70), ("t escritura ultimo (us)", 74),
               ("escrituras medidas", 78)]

def leer_diag(s):
    if s[:4] != b'DIAG' or s[510:512] != b'\x55\xAA':
        return None
    d = {n: struct.unpack('>I', s[o:o+4])[0] for n, o in CAMPOS_DIAG}
    d["ID"] = struct.unpack('>H', s[4:6])[0]
    return d

def imprimir_diag(d):
    print("--- Sector de diagnostico (nodo %d) ---" % d["ID"])
    for n, _ in CAMPOS_DIAG:
        print("  %-28s %d" % (n, d[n]))

def revisar_diag(d, n_analizados, pps):
    """Criterios de aceptacion sobre los contadores del nodo."""
    ceros = ["overflow de bufer", "errores SD", "errores checksum", "discrepancias pulso/trama",
             "saltos de ts en tramas", "bytes fuera de trama", "tramas truncadas", "errores UART"]
    print("\n--- Criterios del diagnostico ---")
    ok_total = True
    def linea(ok, texto):
        nonlocal ok_total
        ok_total &= ok
        print("  [%s] %s" % ("OK " if ok else "FALLA", texto))
    for c in ceros:
        linea(d[c] == 0, "%s = %d (esperado 0)" % (c, d[c]))
    linea(d["sectores llenados"] == d["sectores escritos"],
          "sectores llenados (%d) == escritos (%d)" % (d["sectores llenados"], d["sectores escritos"]))
    linea(d["sectores escritos"] == n_analizados,
          "sectores escritos (%d) == sectores de datos analizados (%d)" % (d["sectores escritos"], n_analizados))
    linea(d["sector_actual"] == SECTOR_INICIAL + d["sectores escritos"],
          "sector_actual (%d) == %d + escritos" % (d["sector_actual"], SECTOR_INICIAL))
    linea(d["pulsos INT1"] == d["tramas validas"],
          "pulsos INT1 (%d) == tramas validas (%d)" % (d["pulsos INT1"], d["tramas validas"]))
    if pps:
        per = 1e6 / pps
        tmax, prom = d["t escritura max (us)"], d["t escritura prom (us)"]
        print("\n--- Margen temporal a %g pulsos/s (periodo %.1f ms) ---" % (pps, per / 1000))
        print("  escritura promedio %.2f ms -> %.1f%% del periodo (debe ser < 100%%; CRC aparte, ~0.5 ms estimado)"
              % (prom / 1000, 100 * prom / per))
        print("  escritura maxima   %.2f ms -> %.2f periodos (el ping-pong tolera < ~2 periodos)"
              % (tmax / 1000, tmax / per))
        print("  margen max = periodo / t_max = %.2f  %s" % (per / tmax if tmax else float('inf'),
              "(tolerable)" if tmax < 2 * per else "(INSUFICIENTE: se esperan overflows en picos lentos)"))
    return ok_total

def analizar(sectores, id_esp=None, modo=None):
    """Valida los sectores de datos. Devuelve (n_errores, n_sectores)."""
    n = len(sectores)
    print("\nSectores de datos analizados: %d" % n)
    if n == 0:
        return 1, 0
    errores, ts_prev, ids, saltos = [], None, set(), 0
    ts0 = struct.unpack('>I', sectores[0][6:10])[0]
    for i, s in enumerate(sectores):
        e = []
        if s[0:4] != b'\xAA\x55\xAA\x55': e.append("cabecera")
        nid = struct.unpack('>H', s[4:6])[0]; ids.add(nid)
        if id_esp is not None and nid != id_esp: e.append("ID %d" % nid)
        ts = struct.unpack('>I', s[6:10])[0]
        if ts_prev is not None and ts != ts_prev + 1:
            e.append("timestamp %d tras %d (delta %d)" % (ts, ts_prev, ts - ts_prev)); saltos += 1
        ts_prev = ts
        if s[510:512] != b'\x55\xAA': e.append("marcador de fin")
        if struct.unpack('>H', s[508:510])[0] != crc16(s[0:508]): e.append("CRC16")
        pay = s[10:508]
        if modo is None and i >= 1:
            modo = "reinicia" if pay[0] == 0 else "continuo"
            print("Patron detectado: %s" % modo)
        base = (i * 498) & 0xFF if (modo == "continuo" and i > 0) else 0
        if bytes((base + k) & 0xFF for k in range(498)) != pay:
            e.append("patron 0-255")
        if e: errores.append((i, ts, e))
    print("IDs presentes: %s" % sorted(ids))
    print("Timestamp: %d -> %d  (%d sectores)" % (ts0, ts_prev, n))
    print("Saltos de timestamp: %d" % saltos)
    if errores:
        print("\nSECTORES CON ERROR: %d" % len(errores))
        for i, ts, e in errores[:30]:
            print("  sector #%d (ts=%d): %s" % (i, ts, ", ".join(e)))
    else:
        print("Cabeceras, ID, CRC16, marcador, patron y timestamp lineal: sin alteraciones.")
    return len(errores), n

# ------------------------------- acceso al disco -----------------------------
def ruta_disco(arg):
    if platform.system() == "Windows" and arg.isdigit():
        return r"\\.\PhysicalDrive" + arg
    return arg

def listar_discos():
    so = platform.system()
    if so == "Windows":
        cmd = ["powershell", "-NoProfile", "-Command",
               "Get-Disk | Select-Object Number,FriendlyName,BusType,"
               "@{n='GB';e={[math]::Round($_.Size/1GB,1)}} | Format-Table -AutoSize"]
    elif so == "Darwin":
        cmd = ["diskutil", "list"]
    else:
        cmd = ["lsblk", "-o", "NAME,SIZE,MODEL,TRAN,RM"]
    try:
        print(subprocess.run(cmd, capture_output=True, text=True).stdout)
    except Exception as ex:
        print("No pude listar los discos (%s)." % ex)
    print("Elige el disco de la tarjeta por su TAMANO (ej. 16 GB, BusType USB/SD). "
          "El script solo lee, pero comprueba que sea el correcto.")

def leer_sectores(ruta, primero, cantidad):
    """Lee 'cantidad' sectores desde 'primero' (acceso de solo lectura, multiplos de 512)."""
    try:
        f = open(ruta, "rb", buffering=0)
    except PermissionError:
        raise ErrorDisco("Sin permiso para abrir %s. Abre VS Code (o la terminal) COMO ADMINISTRADOR; "
                         "en Linux/macOS usa sudo." % ruta)
    except OSError as ex:
        raise ErrorDisco("No pude abrir %s: %s" % (ruta, ex))
    datos = bytearray()
    with f:
        f.seek(primero * SECTOR)
        restante = cantidad
        while restante > 0:
            k = min(restante, 128)                      # bloques de 64 KB
            bloque = f.read(k * SECTOR)
            if not bloque:
                break
            datos += bloque
            restante -= len(bloque) // SECTOR
    return bytes(datos)

# --------------------------------------------------------------------------------
def ejecutar(disco=None, archivo=None, id_esp=None, pps=None, guardar=None, modo=None, maximo=5000):
    """Verifica una tarjeta (disco) o un volcado (archivo). Devuelve 0 si todo esta OK."""
    diag = None
    if disco:
        ruta = ruta_disco(disco)
        s_diag = leer_sectores(ruta, SECTOR_DIAG, 1)
        if len(s_diag) < SECTOR:
            raise ErrorDisco("No pude leer el sector %d de %s." % (SECTOR_DIAG, ruta))
        diag = leer_diag(s_diag)
        if diag:
            imprimir_diag(diag)
            n = diag["sector_actual"] - SECTOR_INICIAL
            if not (0 < n <= 200000):
                raise ErrorDisco("El diagnostico trae sector_actual=%d (incoherente)." % diag["sector_actual"])
            datos = s_diag + leer_sectores(ruta, SECTOR_INICIAL, n)
        else:
            print("AVISO: el sector %d no es un sector de diagnostico (el tren no termino o la "
                  "tarjeta no tiene datos de la Fase 4). Se leen hasta %d sectores desde el %d."
                  % (SECTOR_DIAG, maximo, SECTOR_INICIAL))
            datos = leer_sectores(ruta, SECTOR_INICIAL, maximo)
        if guardar:
            open(guardar, "wb").write(datos)
            print("Volcado guardado en %s (%d bytes)" % (guardar, len(datos)))
    else:
        datos = open(archivo, "rb").read()

    if len(datos) % SECTOR:
        print("AVISO: el volcado no es multiplo de 512 bytes")
    sectores = [datos[i:i+SECTOR] for i in range(0, len(datos) - len(datos) % SECTOR, SECTOR)]

    if sectores and sectores[0][:4] == b'DIAG':          # volcado desde el sector 2047
        if diag is None:
            diag = leer_diag(sectores[0])
            if diag: imprimir_diag(diag)
        sectores = sectores[1:]
    while sectores and (not any(sectores[-1]) or all(b == 0xFF for b in sectores[-1])):
        sectores.pop()

    n_err, n = analizar(sectores, id_esp, modo)
    ok_diag = True
    if diag:
        ok_diag = revisar_diag(diag, n, pps)

    print()
    if n_err == 0 and n > 0 and ok_diag:
        print("RESULTADO: OK")
        return 0
    print("RESULTADO: HAY PROBLEMAS (revisa los puntos marcados arriba)")
    return 1

def modo_vscode():
    """Modo sin argumentos (boton 'Run Python File' de VS Code): pregunta lo necesario."""
    print("=== Verificador Fase 4 (solo lectura) ===")
    if not es_administrador():
        print("AVISO: no estas como administrador. En Windows abre VS Code con "
              "'Ejecutar como administrador'; si no, no podre leer la tarjeta.\n")
    primera = True
    while True:
        disco = DISCO if (primera and DISCO) else ""
        if not disco:
            listar_discos()
            disco = input("Numero de disco de la tarjeta (Windows: 'Number'; o ruta /dev/...): ").strip()
        if primera and ID_NODO is not None:
            id_esp = ID_NODO
        else:
            t = input("ID del nodo de esta tarjeta (1 o 2, Enter = no comprobar): ").strip()
            id_esp = int(t) if t.isdigit() else None
        guardar = None
        if GUARDAR_VOLCADO:
            nombre = "nodo%s.bin" % (id_esp if id_esp is not None else "X")
            guardar = os.path.join(os.path.dirname(os.path.abspath(__file__)), nombre)
        print()
        try:
            ejecutar(disco=disco, id_esp=id_esp, pps=PPS, guardar=guardar)
        except ErrorDisco as ex:
            print("ERROR:", ex)
        primera = False
        if input("\n¿Verificar otra tarjeta? (s/n): ").strip().lower() != "s":
            break
    input("\nPresiona Enter para salir...")
    return 0

def main():
    if len(sys.argv) == 1:
        return modo_vscode()
    ap = argparse.ArgumentParser(description="Verificador Fase 4 (solo lectura)")
    ap.add_argument("archivo", nargs="?", help="volcado .bin copiado con HxD")
    ap.add_argument("--disco", help="numero de disco (Windows) o ruta (/dev/sdX, /dev/rdiskN)")
    ap.add_argument("--listar", action="store_true", help="lista los discos disponibles")
    ap.add_argument("--id", type=int, default=None, help="ID de nodo esperado")
    ap.add_argument("--pps", type=float, default=None, help="pulsos por segundo del ensayo (calcula el margen)")
    ap.add_argument("--max", type=int, default=5000, help="maximo de sectores a leer si no hay diagnostico")
    ap.add_argument("--guardar", help="guarda en un .bin lo leido de la tarjeta")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--continuo", action="store_true")
    g.add_argument("--reinicia", action="store_true")
    a = ap.parse_args()
    modo = "continuo" if a.continuo else "reinicia" if a.reinicia else None
    if a.listar:
        listar_discos(); return 0
    if not a.disco and not a.archivo:
        ap.print_help(); return 2
    try:
        return ejecutar(a.disco, a.archivo, a.id, a.pps, a.guardar, modo, a.max)
    except ErrorDisco as ex:
        print("ERROR:", ex)
        return 1

if __name__ == "__main__":
    sys.exit(main())