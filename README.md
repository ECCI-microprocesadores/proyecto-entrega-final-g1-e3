[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=19632534&assignment_repo_type=AssignmentRepo)
# Proyecto final

## Integrantes
[JHON ALEXANDER CUADROS LAIS](https://github.com/JhonCuadros)

[MIGUEL ANGEL CUERVO](https://github.com/MiguelAcuervo)

## Nombre del proyecto: 

### **CARRO SEGUIDOR DE LINEA**

## Documentación

Este proyecto implementa un carro seguidor de línea utilizando un microcontrolador PIC18F45K22, el controlador de motores L298N y sensores TCRT5000. A continuación, se ofrece una explicación detallada del código y la lógica implementada en cada parte del sistema:

## **Archivo `newmain.c`**

### **Inclusión de bibliotecas**

```c
#include <xc.h>
#include "config.h"
#include "lcd.h"
#include "control.h"
```

- `xc.h`: Librería principal de MPLAB X para trabajar con periféricos del PIC18F45K22.

- `config.h`: Contiene la configuración del microcontrolador (fuses, reloj, etc.).

- `lcd.h`: Encabezado para el manejo del display LCD.

- `control.h`: Encabezado con las funciones de control de motores.

### **Definición de constantes y macros**

```c
#define _XTAL_FREQ 64000000  // Frecuencia de operación del PIC (64 MHz)
#define LS PORTBbits.RB0     // Sensor izquierdo conectado a RB0
#define RS PORTBbits.RB1     // Sensor derecho conectado a RB1
```

Estas directivas permiten usar etiquetas legibles para referirse a las entradas de los sensores.

### **Función `main()`**

La función principal contiene cuatro secciones clave:

#### **a. Configuración inicial del sistema**

```c
OSCCON = 0x70;  // Configura el oscilador interno a 16 MHz
OSCTUNEbits.PLLEN = 1;  // Habilita el PLL (x4) para obtener 64 MHz
```

Esto configura el reloj del sistema para que trabaje a 64 MHz, lo que es fundamental para una ejecución rápida y para que los retardos con __delay_ms sean precisos.

#### **b. Inicialización del LCD y pines**

```c
LCD_Init();        // Inicializa el LCD
LCD_Clear();       // Limpia la pantalla
LCD_SetCursor(1,0);
LCD_String("Robot Seguidor");
__delay_ms(2000);  // Mensaje de bienvenida
LCD_Clear();
```

- `LCD_Init()`: Inicializa la comunicación entre el PIC y el display LCD. Esta función configura los pines de control (RS, EN) y de datos (D4-D7) del LCD como salidas, define el modo de operación de 4 bits, y prepara el LCD para comenzar a recibir comandos o texto.

- `LCD_Clear()`: Envía un comando al LCD para borrar la pantalla y llevar el cursor a la posición inicial (línea 1, columna 0).

- `LCD_SetCursor(1,0):` Coloca el cursor en la línea 1, columna 0 del LCD. Recordemos que los LCDs suelen ser de 16x2, y esta función asegura que el texto se imprima en la ubicación deseada.

- `LCD_String("Robot Seguidor")`: Imprime la cadena de texto "Robot Seguidor" en la pantalla LCD a partir de la posición indicada anteriormente.

- `__delay_ms(2000)`: Espera 2000 milisegundos (2 segundos) para que el usuario pueda leer el mensaje antes de que el sistema comience a funcionar.

- `Segundo LCD_Clear()`: Borra el mensaje de bienvenida para dejar el LCD listo para mostrar datos de operación (como "Izquierda", "Derecha", "Adelante", etc.) durante la ejecución del lazo principal.

> **Importancia:** El LCD actúa como una interfaz de usuario que permite visualizar en tiempo real la decisión del robot, facilitando tanto el seguimiento del comportamiento como la depuración del sistema.

#### **Configuración de los puertos del microcontrolador**

```c
TRISD = 0x00;   // Puerto D como salida
TRISB = 0xFF;   // Puerto B como entrada
ANSELB = 0x00;  // Desactiva funciones analógicas en RB0 y RB1
```
**`TRISD = 0x00;`**

### **¿Cual es su función?**

- Configura todos los pines del Puerto D como salidas digitales.

**Justificación técnica:**

- En el PIC18F45K22, cada puerto tiene un registro llamado TRISx (Tri-State Register) donde:

  - `0` = salida

  - `1` = entrada

    Entonces:

  - `TRISD = 0x00` → Binario: `00000000`

    → Todos los bits del puerto D son salidas.

**¿Para qué se usa el Puerto D en este proyecto?**

El puerto D se utiliza para enviar señales digitales a los motores a través del puente H L298N:

- Por ejemplo, se podrían usar pines como RD0 y RD1 para controlar el motor izquierdo, y RD2 y RD3 para el motor derecho.

- El estado de estos pines (alto o bajo) determina la dirección del giro del motor (adelante, atrás, detenerse).

### 2. Línea: `TRISB = 0xFF;`

### ¿Qué hace?
Configura todos los pines del Puerto B como entradas digitales.

### Explicación técnica:
- `TRISB = 0xFF` ➞ Binario: `11111111`
  - → Todos los bits del puerto B están configurados como entradas.

### Uso específico en el proyecto:
- En este diseño solo se utilizan RB0 y RB1 como entradas para los sensores TCRT5000 (sensor izquierdo y sensor derecho, respectivamente).
- Los otros pines del puerto B (RB2 a RB7) no se usan, pero se mantienen como entradas por seguridad y simplicidad.

> **Nota:** Aunque se podrían configurar individualmente solo RB0 y RB1 como entrada (`TRISB = 0b00000011` o `TRISB = 0x03`), usar `0xFF` generaliza el comportamiento y evita conflictos si se conectan sensores adicionales en el futuro.

###  **3. Línea: `ANSELB = 0x00;`**

### ¿Qué hace?
Desactiva las funciones analógicas de todos los pines del Puerto B, forzando su funcionamiento como entradas/salidas digitales.

### Motivación técnica:
- En el PIC18F45K22, muchos pines del puerto B tienen funciones duales:
  - Digital (entrada/salida normal)
  - Analógica (entrada para el conversor A/D)

- Al inicio del programa, por defecto, algunos pines pueden estar configurados como analógicos, lo cual impediría leer correctamente señales digitales como las que emiten los sensores TCRT5000.

### Solución:
- Establecer `ANSELB = 0x00` garantiza que todos los pines del puerto B operen en modo digital.

> **Si no se desactiva el modo analógico**, el microcontrolador puede interpretar señales digitales como ruido o no responder adecuadamente al leer niveles lógicos.

### **¿Por qué es esencial esta configuración?**

- Permite al PIC leer correctamente los sensores que guían al robot.

- Permite al PIC controlar los motores que definen el movimiento (adelante, izquierda, derecha).

- Garantiza que no haya conflictos eléctricos ni errores de lectura/escritura por configuraciones incorrectas de entrada/salida.

### **c. Configuración de PWM**

```c
setupPWM();  // Inicializa el módulo CCP para generar PWM
```
La función setupPWM() se encuentra en control.c y configura los módulos CCP1 y CCP2 para generar señales PWM en RC2 y RC1 respectivamente, controlando así la velocidad de los motores.

### **d. Lazo Principal de Ejecución `while(1)`**

Dentro del lazo infinito while (1), se encuentra la lógica central del programa, la cual se encarga de realizar la lectura de los sensores infrarrojos, aplicar un filtro para estabilizar las lecturas y tomar decisiones para el control del movimiento del carro seguidor de línea. A continuación se detalla cada parte del proceso:

#### **1. Filtrado por mayoría para estabilizar lecturas**
Se realiza una lectura repetitiva de los sensores izquierdo (LS) y derecho (RS) en un ciclo for de 5 iteraciones. En cada iteración, se verifica si el sensor detecta la línea (valor lógico alto). Se utiliza un contador (countL para el izquierdo y countR para el derecho) para acumular cuántas veces se detecta la línea en las cinco muestras:

```c
unsigned char i;
unsigned char countL = 0;
unsigned char countR = 0;

for (i = 0; i < 5; i++) {
    if (LS) countL++;
    if (RS) countR++;
    __delay_us(read_time); // Retardo entre muestras para evitar lecturas instantáneas consecutivas
}
```
Este mecanismo actúa como un filtro antirruido básico, reduciendo los errores causados por pequeñas fluctuaciones o interferencias eléctricas.

**Explicación Linea del codigo:**

- `unsigned char i;`

  Se declara la variable i como un entero sin signo de 8 bits, que actuará como contador del bucle for. Su rango es de 0 a 255, suficiente para una pequeña cantidad de iteraciones como en este caso.

- `unsigned char countL = 0;`

  Se inicializa el contador countL que se encargará de contar cuántas veces el sensor izquierdo (LS) detecta la línea negra durante las 5 lecturas.

- `unsigned char countR = 0;`

  De forma análoga, se inicializa countR para el sensor derecho (RS).

### **`Bucle for:`**

```c
for (i = 0; i < 5; i++) 
```
Este bucle repetirá su contenido cinco veces, lo cual significa que se tomarán 5 muestras consecutivas de los sensores en un corto intervalo de tiempo. Esto ayuda a filtrar variaciones momentáneas o ruido eléctrico.

**Cuerpo del bucle:**

```c
if (LS) countL++;
```
Se evalúa si el sensor izquierdo (LS) está activado (por ejemplo, si detecta una superficie negra reflejante). Si es así, se incrementa countL en uno.

```c
if (RS) countR++;
```
Lo mismo para el sensor derecho (RS). Si detecta línea, se incrementa countR.

```c
__delay_us(read_time);
```
Se introduce un pequeño retardo en microsegundos (valor definido en la constante read_time) para evitar tomar las cinco muestras de forma instantánea, lo que daría lecturas redundantes. Este retardo mejora el comportamiento del filtro al permitir pequeñas variaciones entre lecturas sucesivas.

#### **Resultado:**

Después de las cinco iteraciones, countL y countR contendrán valores entre 0 y 5 que representan cuántas veces cada sensor detectó línea en esas cinco muestras. Estos contadores se usan luego para tomar una decisión lógica con base en la mayoría (es decir, si 3 o más lecturas fueron positivas, se considera que efectivamente se detectó línea).

#### **¿Por qué se hace esto?**

El objetivo es reducir errores de detección debidos a ruido eléctrico o superficies irregulares. Por ejemplo, si un sensor da una lectura incorrecta momentánea, el filtro por mayoría probablemente la ignorará si las otras lecturas fueron correctas.

**Evaluación de valores filtrados**

Se determina el valor final de cada sensor mediante una condición de mayoría. Si al menos 3 de las 5 lecturas indican la presencia de línea, se considera que efectivamente se ha detectado:

```c
unsigned char leftValue = (countL >= 3) ? 1 : 0;
unsigned char rightValue = (countR >= 3) ? 1 : 0;
```
**Explicación general**

Se está utilizando el operador condicional ternario `(? :)`, el cual es una forma abreviada de una estructura `if-else`. Su formato general es:

```c
condición ? valor_si_verdadero : valor_si_falso;
```
Este operador evalúa una condición lógica. Si es verdadera, se asigna el primer valor; si es falsa, se asigna el segundo.

**Línea por línea:**

- `unsigned char leftValue = (countL >= 3) ? 1 : 0`;

  - countL representa el número de veces que el sensor izquierdo detectó línea en las 5 lecturas.

  - La condición `countL >= 3` significa: “¿Detectó línea en al menos 3 de las 5 lecturas?”

  - Si sí (es decir, se obtuvo mayoría), se asigna 1 a la variable leftValue, lo que indica una detección afirmativa por parte del sensor izquierdo.

  - Si no, se asigna 0, indicando no detección.

- `unsigned char rightValue = (countR >= 3) ? 1 : 0`;

  - Exactamente el mismo razonamiento, pero aplicado al sensor derecho (RS).

  - Se interpreta como: “¿El sensor derecho detectó línea en al menos 3 de 5 lecturas?”

**¿Por qué se utiliza este enfoque?**

Este mecanismo actúa como una forma sencilla de filtrado digital de señales binarias. No basta con que el sensor indique una vez que hay línea: se necesitan al menos 3 lecturas afirmativas de 5 posibles para que el sistema lo considere una detección válida.

Esto es útil para:

- Minimizar errores causados por ruido eléctrico o interferencias.

- Suavizar lecturas en superficies con imperfecciones o bordes difusos.

- Evitar reacciones erráticas del robot si una lectura puntual no refleja la realidad.


## Diagramas


## Conclusiones


<!-- Crear una carpeta src e incluir en ella los códigos y/o el proyecto de mplab-->
