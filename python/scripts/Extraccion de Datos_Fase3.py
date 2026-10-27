import os
import sys

# Ruta asignada de tu Administración de Discos
ruta_tarjeta_sd = r'\\.\PhysicalDrive1'  
archivo_salida = "cabeceras_32bytes.txt"

# === PARÁMETROS DE EXTRACCIÓN AJUSTADOS ===
bytes_por_fila = 32       # <--- Cambiado de 16 a 32 bytes por sector
tamaño_sector = 512       # Tamaño físico estándar por sector
total_sectores = 3600      # Cantidad de sectores consecutivos
offset_inicial = 0x100000 # Sector 2048 (1,048,576 bytes)

try:
    with open(ruta_tarjeta_sd, "rb") as sd_card, open(archivo_salida, "w", encoding="utf-8") as f_out:
        print(f"Abriendo tarjeta Micro SD física (Disco 1)...")
        print(f"Extrayendo los primeros {bytes_por_fila} bytes de {total_sectores} sectores...")
        
        # Nos posicionamos en el offset inicial (Sector 2048)
        sd_card.seek(offset_inicial)
        
        for i in range(total_sectores):
            # Lee los primeros 32 bytes del sector actual
            data_bytes = sd_card.read(bytes_por_fila)
            if not data_bytes:
                print("Fin de los datos del dispositivo alcanzado de forma inesperada.")
                break
                
            # Convierte los 32 bytes a texto hexadecimal legible
            hex_string = " ".join(f"{b:02X}" for b in data_bytes)
            
            # Escribe la línea en el archivo de texto
            f_out.write(f"Valores Hexadecimales: {hex_string}\n")
            
            # Salta el resto del sector. Al leer 32 bytes, ahora salta exactamente 480 bytes 
            # (512 - 32 = 480) para caer justo al inicio del siguiente sector.
            sd_card.seek(tamaño_sector - bytes_por_fila, 1)

    print(f"\n¡Extracción exitosa!")
    print(f"Se ha creado el archivo de texto: '{archivo_salida}' con bloques de 32 bytes.")

except PermissionError:
    print("\n[ERROR] Acceso denegado. Recuerda abrir tu VS Code o Terminal como ADMINISTRADOR.")
except Exception as e:
    print(f"\n[ERROR] Ocurrió un problema: {e}")
