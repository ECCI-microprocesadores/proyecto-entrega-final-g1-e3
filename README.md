[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=19632534&assignment_repo_type=AssignmentRepo)
# Proyecto final

## Integrantes
[JHON ALEXANDER CUADROS LAIS](https://github.com/JhonCuadros)

[MIGUEL ANGEL CUERVO](https://github.com/MiguelAcuervo)

## Nombre del proyecto: 

### **CARRO SEGUIDOR DE LINEA**

![DIAGRAMA](/IMAGENES/carro_2.jpg)

# Objetivos

## General

Desarrollar un carro seguidor de línea controlado por el microcontrolador PIC18F45K22, el cual utiliza dos sensores ópticos TCRT5000 para detectar la trayectoria marcada sobre una superficie. El sistema está diseñado para tomar decisiones de movimiento (adelante, izquierda, derecha o detenerse) según las lecturas obtenidas por los sensores, mediante un algoritmo de control embebido. El movimiento del vehículo es gestionado a través de un puente H (L298N), que permite el control bidireccional de los motores de corriente continua. Adicionalmente, el vehículo incorpora una pantalla LCD 20x4 que permite visualizar en tiempo real la dirección en la que se desplaza y los valores de velocidad en RPM, brindando retroalimentación al usuario sobre el comportamiento del sistema.

## Especificos

- Implementar un sistema de seguimiento de línea mediante el uso de dos sensores infrarrojos TCRT5000, capaces de detectar el contraste entre la línea guía y el fondo.
Controlar los motores de corriente continua del vehículo mediante un puente H tipo L298N, permitiendo el movimiento en distintas direcciones según las señales procesadas.

- Programar el microcontrolador PIC18F45K22 en el entorno MPLAB X IDE utilizando lenguaje C, con el fin de interpretar las señales de los sensores, generar las señales PWM necesarias para el control de velocidad y dirección de los motores, y mostrar información en tiempo real en una pantalla LCD.

# Documentación

Este proyecto implementa un carro seguidor de línea utilizando un microcontrolador PIC18F45K22, el controlador de motores L298N y sensores TCRT5000. A continuación, se ofrece una explicación detallada del código y la lógica implementada en cada parte del sistema:

## **Archivo `newmain.c`**

### **Configuración del microcontrolador**

```c
#pragma config FOSC = INTIO67  
#pragma config PLLCFG = ON
#pragma config WDTEN = OFF
#pragma config BOREN = SBORDIS
```
Estos `#pragma config` configuran los fuses del microcontrolador. No son registros directamente, pero controlan cómo inicia y se comporta el PIC.

- `FOSC = INTIO67:` Usa el oscilador interno, habilitando RA6 y RA7 como I/O.

- `PLLCFG = ON:` Activa el PLL para multiplicar la frecuencia interna.

- `WDTEN = OFF:` Desactiva el Watchdog Timer (evita reinicios automáticos).

- `BOREN = SBORDIS:` Desactiva el Brown-out Reset (reinicio por baja tensión).

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

### **Inicialización y declaracion de variables**

```c
unsigned char duty = 37;
signed int read_time = 100;;
```

**¿Qué hace?**

```c
unsigned char duty = 37;
```
Declara e inicializa una variable duty de tipo `unsigned char` con valor 37.

**¿Para qué sirve?**

duty representa el ciclo de trabajo (duty cycle) del PWM, que controla la velocidad de los motores.

- Un valor de 37 significa que el PWM está activo el 37% del tiempo y apagado el 63%.

- Esto controla la velocidad media del motor.


**¿Qué hace?**

```c
signed int read_time = 100;
```

Declara una variable read_time de tipo signed int (entero con signo), con valor inicial 100.

**¿Para qué sirve?**

Es el tiempo de retardo en microsegundos entre cada lectura de los sensores en el filtro de mayoría:

```c
__delay_us(read_time);
```

Esto:

- Da un pequeño margen de tiempo entre lecturas repetidas.

- Ayuda a evitar ruido y lecturas falsas de los sensores.

- Permite que el filtro simple (leer 5 veces y tomar mayoría) sea más estable.

### **Función `main()`**

La función principal contiene tres secciones clave:

### **Parte 1: Configuración inicial**

```c
void main(void) {
    // Configurar oscilador interno a 64 MHz
    OSCCON = 0b01110000;
    OSCTUNEbits.PLLEN = 1;
```

**¿Qué hace?**

- `OSCCON = 0b01110000;` configura el reloj interno del PIC a 16 MHz.

- `OSCTUNEbits.PLLEN = 1;` activa el PLL (Phase-Locked Loop), multiplicando la frecuencia a 64 MHz (16 MHz * 4).

Esto asegura una mayor velocidad de procesamiento, importante para temporización precisa y control PWM.

**¿Qué hace?**

```c
    LCD_Init();
    __delay_ms(2000);
    LCD_Clear();
```

- `LCD_Init();` inicializa el módulo LCD para mostrar texto.

- `__delay_ms(2000);` espera 2 segundos para asegurar que el LCD esté listo.

- `LCD_Clear();` limpia cualquier contenido previo en la pantalla.

### **Parte 2: Configuración de pines**

```c
    // Configurar pines de motores como salida
    TRISD &= ~(0x0F);  // RD0-RD3
    LATD &= ~(0x0F);

```

**¿Qué hace?**

- `TRISD` controla la dirección de los pines del puerto D.

- `~(0x0F)` limpia los bits 0-3 (RD0-RD3), configurándolos como salidas digitales (para controlar los motores).

- `LATD &= ~(0x0F);` asegura que esas salidas estén inicialmente en bajo (apagadas).

#### **Configuración de los puertos del microcontrolador**
```c
    // Configurar sensores como entrada digital
    TRISBbits.TRISB0 = 1;  // RB0
    TRISBbits.TRISB1 = 1;  // RB1
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
```

- Configura RB0 y RB1 como entradas digitales.

- `ANSELBbits.ANSB0/1 = 0;` desactiva el modo analógico, ya que se requiere lectura digital de los sensores de línea.

**¿Qué hace?**
```c
        setupPWM();
```
- Inicializa los módulos PWM (por ejemplo, CCP1 y CCP2) para controlar la velocidad de los motores usando el valor duty.

### **Parte 3: Bucle principal `(while(1))`**

```c
    char estadoAnterior[10] = "";
```
**¿Qué hace?**

Almacena el último estado mostrado en el LCD, para evitar escribir repetidamente el mismo texto.

### **Lectura de sensores con filtro**
```c
        while (1) {
        unsigned char i;
        unsigned char countL = 0;
        unsigned char countR = 0;

        for (i = 0; i < 5; i++) {
            if (LS) countL++;
            if (RS) countR++;
            __delay_us(read_time);
        }

        unsigned char leftValue = (countL >= 3) ? 1 : 0;
        unsigned char rightValue = (countR >= 3) ? 1 : 0;
```

**Variables de conteo**

```c
    unsigned char i;
    unsigned char countL = 0;
    unsigned char countR = 0;
```
**¿Qué hace?**

- `i:` variable usada como índice para el ciclo for.

- `countL:` contador de lecturas positivas (valor lógico alto) del sensor izquierdo (LS).

- `countR:` contador de lecturas positivas del sensor derecho (RS).

Ambos contadores comienzan en 0 en cada iteración del while, ya que se hace una nueva evaluación de los sensores.

### Lectura múltiple de sensores (con filtro)**

```c
    for (i = 0; i < 5; i++) {
        if (LS) countL++;
        if (RS) countR++;
        __delay_us(read_time);
    }
```
**¿Qué hace?**

Este bucle se ejecuta 5 veces seguidas`(de i = 0 a i = 4)`. Cada vez:

1. Lee los sensores:

   - LS (RB0) y RS (RB1) son definidos como:

    ```c
    #define LS PORTBbits.RB0
    #define RS PORTBbits.RB1
   ```
   Por tanto, cada `if (LS)` o `if (RS)` evalúa si el pin está en alto (`1`).

2. Si el valor es alto (`1`), el contador respectivo (`countL` o `countR`) se incrementa.

3. Después de cada lectura, se realiza una pequeña pausa con:

   ```c
   __delay_us(read_time);  // típico valor: 100 us
   ```
   Esto espacia las lecturas y ayuda a evitar lecturas inestables o rebotes eléctricos.

### **Filtro de mayoría: decisión final**

```c
    unsigned char leftValue = (countL >= 3) ? 1 : 0;
    unsigned char rightValue = (countR >= 3) ? 1 : 0;
   ```

**¿Qué hace?**

Este es un operador ternario que convierte los conteos en una decisión binaria final (`1 = línea detectada`, `0 = no detectada`):

- Si el sensor fue leído como "activo" al menos 3 de 5 veces, se considera válido y se asigna 1.

- Si no alcanzó 3 lecturas válidas, se asume que no se detectó línea y se asigna 0.

**Resultado:**

- `leftValue = 1` → Sensor izquierdo detectó línea.

- `leftValue = 0` → No detectó línea.

- Lo mismo para `rightValue`.

**¿Por qué es útil esta técnica?**

**Sin filtro:**

Un solo cambio en el sensor (por ejemplo, reflejo, sombra, rebote eléctrico) puede hacer que el carro tome una decisión equivocada.

**Con filtro de mayoría:**

El sistema necesita 3 lecturas consistentes de 5 para cambiar de dirección. Esto elimina errores transitorios o falsos positivos/negativos.

### **Decisiones de movimiento**

```c
if (leftValue == 1 && rightValue == 0) {
            turnLeft(duty);
            if (strcmp(estadoAnterior, "Izquierda") != 0) {
                LCD_Clear();
                LCD_SetCursor(1, 0);
                LCD_String("Izquierda");
                strcpy(estadoAnterior, "Izquierda");
            }
        }
```
La sección decide qué debe hacer el robot (girar, avanzar o detenerse) según lo que detectan los sensores de línea. También actualiza el mensaje en la pantalla LCD solo si el estado ha cambiado (para evitar parpadeos innecesarios).

### Recordatorio: ¿qué significan `leftValue` y `rightValue`?

Estos valores vienen de la lógica de filtro anterior:

| Valor | Significado para sensor     |
|:-----:|:-----------------------------|
| `1`   | Sensor detecta línea         |
| `0`   | Sensor no detecta línea      |

### **1. Girar a la izquierda**

```c
if (leftValue == 1 && rightValue == 0) {
    turnLeft(duty);
```
- El sensor izquierdo ve la línea y el derecho no → la línea está a la izquierda.

- El carro debe girar a la izquierda.

- `turnLeft(duty)` controla los motores para ese giro con cierto ciclo de trabajo (PWM).

**Actualización de la pantalla:**

```c
    if (strcmp(estadoAnterior, "Izquierda") != 0) {
        LCD_Clear();
        LCD_SetCursor(1, 0);
        LCD_String("Izquierda");
        strcpy(estadoAnterior, "Izquierda");
    }
```
- Compara el estado anterior con el actual.

- Solo si ha cambiado a “Izquierda”, borra y actualiza el LCD.

- Esto evita que el mensaje se refresque inútilmente y elimina parpadeo.

### **2. Girar a la derecha**

```c
} else if (leftValue == 0 && rightValue == 1) {
    turnRight(duty);
```
- El sensor derecho detecta línea, el izquierdo no → la línea está a la derecha.

- Se llama a `turnRight(duty)`.

**Actualización LCD (misma lógica):**
```c
    if (strcmp(estadoAnterior, "Derecha") != 0) {
        LCD_Clear();
        LCD_SetCursor(1, 0);
        LCD_String("Derecha");
        strcpy(estadoAnterior, "Derecha");
    }
```
### **3. Seguir recto**

```c
} else if (leftValue == 0 && rightValue == 0) {
    forward(duty);
```
- Ningún sensor ve línea → significa que la línea está al centro, entre los sensores.

- Se avanza recto.

**Actualización LCD (misma lógica):**
```c
    if (strcmp(estadoAnterior, "Recto") != 0) {
        LCD_Clear();
        LCD_SetCursor(1, 0);
        LCD_String("Recto");
        strcpy(estadoAnterior, "Recto");
    }
```
### **4. Detenerse (sin línea)**

```c
} else {
    stopMotors();
```
- Este caso ocurre cuando ambos sensores detectan línea (leftValue == 1 && rightValue == 1).

- En algunos diseños de seguidor de línea, eso indica que el robot salió de pista o está sobre una intersección negra.

- Llama a `stopMotors()` para detener el movimiento.

**Actualización LCD (misma lógica):**

```c
    if (strcmp(estadoAnterior, "No linea") != 0) {
        LCD_Clear();
        LCD_SetCursor(1, 0);
        LCD_String("No linea");
        strcpy(estadoAnterior, "No linea");
    }
```
### **Retardo pequeño**

```c
} else {
    __delay_ms(1);
```
- Se agrega un pequeño delay para:

  - Dar tiempo a que el hardware reaccione.

  - Evitar rebotes o lecturas demasiado rápidas.

  - Reduce la carga de procesamiento.

### **¿Por qué verificar el estado antes de actualizar el LCD?**
```c
if (strcmp(estadoAnterior, "X") != 0)
```
Evita:

- Borrar y escribir la misma palabra cada vez.

- Flickering (parpadeo molesto en pantallas LCD).

- Pérdida de rendimiento por escrituras innecesarias.

## **Archivo: `control.c`**

El archivo control.c es fundamental en este proyecto porque contiene todas las funciones encargadas del control de movimiento del carro, es decir, es el cerebro lógico que define cómo y cuándo se mueven los motores.


```c
#include "control.h"
```
Esto indica que hay un archivo control.h donde están declaradas las funciones, pines, macros o variables utilizadas aquí.

### **1. setupPWM(void)**

```c
void setupPWM(void) {
    TRISCbits.TRISC2 = 0;
    CCP1CON = 0b00001100;
    CCPR1L = 0;

    TRISCbits.TRISC1 = 0;
    CCP2CON = 0b00001100;
    CCPR2L = 0;

    PR2 = 60;
    T2CON = 0b00000111; // Prescaler 1:16, Timer2 ON
}
```
Configura los módulos CCP1 y CCP2 del PIC para generar PWM (Pulse Width Modulation) en los pines RC2 y RC1, que van conectados a los drivers de los dos motores.

- `CCP1CON` y `CCP2CON` activan el modo PWM (bits 1100 → PWM mode).

- `CCPR1L` y `CCPR2L` controlan el ciclo de trabajo del PWM (duty cycle).

- `PR2 = 60` define el período del PWM junto con el Timer2.

- `T2CON = 0b00000111`:

  - `Bit 2 = 1:` enciende `Timer2`.

  - `Bits 1:0 = 11:` prescaler 1:16.

### Fórmula de Período PWM

De acuerdo con la imagen, el **período de la señal PWM** está dado por la fórmula:

**Usamos la fórmula:**

$$
\text{Periodo}_{\text{PWM}} = (\text{PR2} + 1) \times 4 \times T_{\text{OSC}} \times \text{Prescaler}
$$

Donde:

- `PR2` es el valor del registro del temporizador 2 que define el período del PWM.
- `T_OSC` es el período del oscilador (por ejemplo, para un cristal de 16 MHz:  '$$T_OSC = 1 / 16 MHz = 62.5 ns$$.
- `Prescaler` es el divisor de frecuencia del Timer2 (por ejemplo, 16).
- El factor **4** es parte del cálculo interno del módulo PWM del PIC.

**Análisis de `setupPWM(void)`**

```c
void setupPWM(void) {
    TRISCbits.TRISC2 = 0;  // RC2 como salida para CCP1 (PWM1)
```
Configura RC2 como salida digital. El pin RC2 se usará para enviar la señal PWM generada por el módulo CCP1.

```c
    CCP1CON = 0b00001100;  // Modo PWM para CCP1
```
Activa el modo PWM en el módulo CCP1 (bits <3:0> = 1100).

```c
    CCPR1L = 0;
```
Inicializa el duty cycle de CCP1 en 0.

```c
    TRISCbits.TRISC1 = 0;  // RC1 como salida para CCP2 (PWM2)
    CCP2CON = 0b00001100;  // Modo PWM para CCP2
    CCPR2L = 0;
```
Lo mismo, pero para el módulo CCP2. RC1 será la salida PWM2.

```c
    PR2 = 60;
```
Este es el valor importante que define el período del PWM.

```c
        T2CON = 0b00000111; // Prescaler 1:16, Timer2 ON
```
Bits:

- `TMR2ON = 1:` Enciende el Timer2.

- `T2CKPS1:T2CKPS0 = 1:1:` `Prescaler = 16`.

### Cálculo del Período y Frecuencia

### Supuestos:

- Oscilador:  

$$
\
F_{OSC} = 16\ \text{MHz} \Rightarrow T_{OSC} = 62.5\ \text{ns}
\
$$

- Prescaler: 16

- `PR2 = 60`

---

### Usamos la fórmula:

$$
Período_{PWM} = (PR2 + 1) \times 4 \times T_{\text{OSC}} \times \text{Prescaler}
$$

$$
= (60 + 1) \times 4 \times 62.5\ \text{ns} \times 16
$$

$$
= 61 \times 4 \times 62.5\ \text{ns} \times 16
$$

$$
= 61 \times 4 \times 1000\ \text{ns} = 61 \times 4 \times 1\ \mu s = 244\ \mu s
$$


### Frecuencia:
$$
\
F_{PWM} = \frac{1}{244\ \mu s} \approx 4.10\ \text{kHz}
\
$$


### **2. setDuty1 y setDuty2**

```c
void setDuty1(unsigned char val) {
    CCPR1L = (val == 0) ? 1 : val;
}
void setDuty2(unsigned char val) {
    CCPR2L = (val == 0) ? 1 : val;
}
```
**¿Qué hacen?**

Asignan el ciclo de trabajo del PWM de los dos canales:

- `val = 0` → en vez de poner 0, se fuerza a 1 para evitar que el motor quede totalmente sin energía (algunos drivers no reaccionan bien con 0).

- El valor afecta directamente la velocidad del motor.


### **Valor del Ciclo Util**

```c
void setDuty1(unsigned char val) {
    CCPR1L = (val == 0) ? 1 : val;
}
```
Aquí el valor val es un número entre 0 y 255, que representa directamente el ciclo útil (Duty Cycle). Entonces puedes calcular el ciclo útil con:

**Cálculo del Duty Cycle**

$$
DutyCycle = \frac{val}{255} \times 100
$$

### Ejemplos prácticos

| `val` | Duty Cycle (%) |
|------:|----------------|
| 0     | $$\frac{0}{255} \times 100 = 0\ $$ |
| 127   | $$\frac{127}{255} \times 100 \approx 49.8\% $$ |
| 255   | $$\frac{255}{255} \times 100 = 100\% $$ |


Y lo mismo para `setDuty2`.

```c
void setDuty2(unsigned char val) {
    CCPR2L = (val == 0) ? 1 : val;
```

### **Funciones de movimiento**

Estas funciones controlan la dirección de giro de los motores según la lógica H-Bridge (puente H).

**`void forward(unsigned char val)`**
```c
IN1 = 1; IN2 = 0;
IN3 = 0; IN4 = 1;
setDuty1(val);
setDuty2(val);
```
**¿Qué hace?**

- Ambos motores giran hacia adelante.

- Se aplica el mismo valor val de PWM a ambos.

- Resultado: el carro avanza en línea recta hacia el frente.

**`void reverseMotor1(unsigned char val)`**

```c
IN1 = 0; IN2 = 1;   // Motor 1 reversa
IN3 = 0; IN4 = 1;   // Motor 2 adelante
setDuty1(val);      // Velocidad motor 1
setDuty2(val);      // Velocidad motor 2
```
**¿Qué hace?**

- Motor 1 gira hacia atrás, motor 2 sigue hacia adelante.

- Ambos tienen misma velocidad.

- Resultado: el carro gira en una curva hacia un lado, o incluso gira sobre su propio eje (dependiendo de la configuración mecánica).

**`void turnRight(unsigned char val)`**

```c
IN1 = 1; IN2 = 0;   // Motor 1 adelante
IN3 = 0; IN4 = 0;   // Motor 2 apagado
setDuty1(val);      // Solo motor 1 activo
setDuty2(0);
```
**¿Qué hace?**

- Solo el motor izquierdo (motor 1) se mueve hacia adelante.

- El motor derecho (motor 2) está apagado.

- Resultado: el carro gira suavemente hacia la derecha.

> Esto da un giro amplio, más útil para maniobras lentas o curvas controladas.

**`void turnLeft(unsigned char val)`**

```c
IN1 = 0; IN2 = 0;   // Motor 1 apagado
IN3 = 0; IN4 = 1;   // Motor 2 adelante
setDuty1(0);
setDuty2(val);      // Solo motor 2 activo
```
**¿Qué hace?**

- Solo el motor derecho (motor 2) gira hacia adelante.

- El motor izquierdo está apagado.

- Resultado: el robot gira suavemente hacia la izquierda.

**`void stopMotors(void)`**

```c
IN1 = 0; IN2 = 0;
IN3 = 0; IN4 = 0;
setDuty1(0);
setDuty2(0);
```
**¿Qué hace?**

- Ambos motores están completamente detenidos.

- Las señales de dirección y PWM están en bajo.

> Esto es un estado de reposo total. El carro no se moverá hasta que se emita otro comando.

## **Archivo `control.h`**

El archivo `control.h` es el archivo de cabecera (header) correspondiente al archivo `control.c`, y cumple una función esencial en la estructura del proyecto: declarar las funciones y definiciones necesarias para controlar los motores, para que puedan ser utilizadas desde otros archivos como `main.c`.

**Configuración del Pic**

```c
#ifndef CONTROL_H
#define CONTROL_H
```
- Estas líneas son directivas de preprocesador que evitan que el archivo se incluya más de una vez.

- `#ifndef` significa “si no está definido”, y `#define CONTROL_H` marca que este archivo ya fue incluido.

- Esto evita errores por redefinición al compilar.

**Incluir libreria**

```c
#include <xc.h>
```
- Incluye el encabezado principal de MPLAB/XC8 que contiene las definiciones necesarias para trabajar con registros, puertos, periféricos, etc. del PIC18F45K22.

- Es necesario para poder usar instrucciones como LATDbits.LATD0.

**Pines de control de motores**

```c
// Pines de control de motores (L298N)
#define IN1 LATDbits.LATD0
#define IN2 LATDbits.LATD1
#define IN3 LATDbits.LATD2
#define IN4 LATDbits.LATD3
```
- Estas líneas definen macros para facilitar el uso de los pines que controlan el puente H L298N.

- En lugar de escribir LATDbits.LATD0, puedes simplemente escribir IN1.

- Mejora la legibilidad y hace que sea más fácil cambiar de pines si fuera necesario (solo editas aquí).

- Estos pines determinan el sentido de giro de los motores.

**Funciones**

```c
void setupPWM(void);
void setDuty1(unsigned char val);
void setDuty2(unsigned char val);
```

Prototipos de funciones que permiten configurar el módulo PWM del PIC y establecer el ciclo de trabajo (duty cycle) de cada motor.

- `setupPWM()` configura los módulos CCP y Timer2.

- `setDuty1()` y `setDuty2()` ajustan la velocidad de los motores mediante PWM.

```c
void forward(unsigned char val);
void turnRight(unsigned char val);
void turnLeft(unsigned char val);
void stopMotors(void);
```

Prototipos de funciones que permiten controlar el comportamiento del movimiento del robot:

- forward(val) → Avanza recto con velocidad val.

- turnRight(val) → Gira a la derecha.

- turnLeft(val) → Gira a la izquierda.

- stopMotors() → Detiene ambos motores.

> Estas funciones están definidas en control.c, pero se declaran aquí para poder usarlas desde cualquier otro archivo del programa.

**Final**

```c
#endif
```
Marca el final de la protección contra inclusiones múltiples.

## **Archivo `lcd.c`**

El archivo lcd.c contiene la implementación de funciones necesarias para controlar un display LCD alfanumérico (como un LCD 16x2 o 20x4) utilizando un modo de 4 bits y conectado a un microcontrolador como el PIC18F45K22.

### **¿Qué hace lcd.c?**

Implementa funciones como:

- Inicializar el LCD (LCD_Init)

- Enviar comandos (LCD_Command)

- Escribir caracteres y cadenas (LCD_Char, LCD_String)

- Posicionar el cursor (LCD_SetCursor)

- Limpiar la pantalla (LCD_Clear)

- Crear caracteres personalizados (LCD_CREATECHAR)

Estas funciones interactúan directamente con los pines de datos y control conectados al LCD.

### **Explicación detallada de cada función**

#### **`void LCD_Enable(void)`**

```c
EN = 1;
__delay_us(5);
EN = 0;
__delay_us(100);
```
- Genera un pulso en el pin EN (Enable) del LCD.

- El LCD detecta los datos presentes cuando EN cambia de 1 a 0.

- Los retardos (__delay_us) aseguran que el LCD tenga tiempo de procesar.

#### **`void LCD_Command(unsigned char cmd)`**

- Envía un comando al LCD (no carácter).

- Se divide el byte cmd en dos mitades porque el LCD está en modo de 4 bits:

  - Primero se mandan los 4 bits altos (cmd >> 4)

  - Luego los 4 bits bajos (cmd & 0x0F)

- RS = 0 indica que es un comando, no datos.

```c
    RS = 0;
    D4 = (cmd >> 4) & 1;
    D5 = (cmd >> 5) & 1;
    D6 = (cmd >> 6) & 1;
    D7 = (cmd >> 7) & 1;
    LCD_Enable();

    D4 = (cmd >> 0) & 1;
    D5 = (cmd >> 1) & 1;
    D6 = (cmd >> 2) & 1;
    D7 = (cmd >> 3) & 1;
    LCD_Enable();

    if (cmd == 0x01 || cmd == 0x02)
        __delay_ms(2);
```
Si el comando es 0x01 (clear) o 0x02 (return home), se agrega un retardo adicional porque son instrucciones lentas. 

La linea

```c
if (cmd == 0x01 || cmd == 0x02)
    __delay_ms(2);
```
dentro de la función LCD_Command(unsigned char cmd) detecta comandos especiales del LCD que requieren más tiempo de procesamiento, y por eso introduce una espera adicional de 2 milisegundos (__delay_ms(2)).

### ¿Qué significan esos comandos?

| Comando | Código | Descripción                                                  | Tiempo típico de ejecución |
|---------|--------|--------------------------------------------------------------|-----------------------------|
| 0x01    | Clear Display | Limpia toda la pantalla, pone el cursor en (0,0)           | ~1.52 ms                    |
| 0x02    | Return Home   | Mueve el cursor al inicio (0,0) sin borrar texto         | ~1.52 ms                    |

> Estos comandos son **más lentos que los demás** (como mover el cursor, encender/apagar el display, etc.), por eso es **obligatorio esperar al menos 1.5 ms** después de enviarlos, para que el LCD no se bloquee o ignore nuevos comandos.

**¿Por qué es importante?**

El LCD necesita tiempo para procesar internamente ciertas instrucciones. Si no se hace esa pausa:

- Puedes ver caracteres extraños

- El texto puede no actualizarse correctamente

- El LCD puede desincronizarse

#### **`void LCD_Char(unsigned char data)`**

```c
    RS = 1;
    D4 = (data >> 4) & 1;
    D5 = (data >> 5) & 1;
    D6 = (data >> 6) & 1;
    D7 = (data >> 7) & 1;
    LCD_Enable();

    D4 = (data >> 0) & 1;
    D5 = (data >> 1) & 1;
    D6 = (data >> 2) & 1;
    D7 = (data >> 3) & 1;
    LCD_Enable();
```

- Similar a LCD_Command, pero `RS = 1` porque es un dato (carácter a mostrar).

- También se envía en dos partes (modo 4 bits).

#### **`void LCD_Clear(void)`**
```c
    LCD_Command(0x01);
    __delay_ms(2);
```

- Envía el comando 0x01, que limpia toda la pantalla.

- Se espera ~2 ms porque esta operación es lenta.

#### **`void LCD_Init(void)`**

```c
    TRISC = 0x00;
    TRISD = 0x00;
    __delay_ms(20);

    RS = 0;
    D4 = 0; D5 = 0; D6 = 1; D7 = 0;  // Comando 0x30 >> 4
    LCD_Enable();
    __delay_ms(5);

    LCD_Enable();
    __delay_us(150);

    LCD_Enable();

    D4 = 0; D5 = 1; D6 = 0; D7 = 0;  // Comando 0x20 >> 4 (modo 4 bits)
    LCD_Enable();

    LCD_Command(0x28);  // LCD 4 bits, 2 líneas, 5x8
    LCD_Command(0x0C);  // Display ON, cursor OFF
    LCD_Command(0x06);  // Incremento automático del cursor
    LCD_Clear();
```

**Inicio de la función que configura e inicializa el LCD.**

```c
  TRISC = 0x00;
  TRISD = 0x00;
```
- Se configuran todos los pines de los puertos C y D como salidas digitales (0 = salida).

- Esto es necesario porque los pines conectados al LCD (RS, EN, D4-D7) necesitan enviar señales desde el PIC hacia el LCD.

```c
  RS = 0;
  D4 = 0; D5 = 0; D6 = 1; D7 = 0;  // Comando 0x30 >> 4
  LCD_Enable();
```
**`RS = 0;`**

- RS (Register Select) es el pin que indica al LCD qué tipo de información se está enviando:

- RS = 0: se está enviando un comando (instrucción al LCD).

- RS = 1: se está enviando datos (un carácter para mostrar, por ejemplo).

Aquí se pone en 0 porque estamos enviando un comando de inicialización.

**`D4 = 0; D5 = 0; D6 = 1; D7 = 0;`**

- Se están configurando los pines de datos D4 a D7 para enviar un nibble (4 bits) al LCD.

- Este valor representa el nibble alto del comando 0x30 (es decir, 0011 0000 en binario).

- Como estamos en modo 4 bits, el comando se envía en dos partes:

  - Primero el nibble alto (0x3 → 0011)

  - Luego el nibble bajo (0x0 → 0000)

- Aquí se está enviando 0x30 >> 4 = 0x03 = 0011, así que los pines se configuran como:

  - D7 = 0

  - D6 = 1

  - D5 = 0

  - D4 = 0

**`LCD_Enable();`**

- Esta función genera un pulso de habilitación en el pin EN:

  - EN = 1 → pequeña espera → EN = 0

- Este pulso hace que el LCD lea el valor de los pines de datos (D4–D7) y lo interprete como un comando.

> Sin este pulso, el LCD no capturaría los datos que están en el bus.

**`__delay_ms(5);`**

- Espera de al menos 5 milisegundos para que el LCD tenga tiempo de procesar el comando anterior.

- Es necesario porque después de este primer comando, el LCD puede tardar más tiempo en responder (según su hoja de datos).


#### **¿Qué hace esta secuencia completa?**

Este bloque:

```c
RS = 0;
D4 = 0; D5 = 0; D6 = 1; D7 = 0;  // Comando 0x30 >> 4
LCD_Enable();
__delay_ms(5);
```
En resumen: se indica que se va a trabajar con el LCD, se le envía un comando de inicialización (0x30), y se espera a que lo procese. Esto prepara al LCD para la comunicación estable antes de cambiar a modo 4 bits.

```c
 LCD_Enable();
    __delay_us(150);
```
Esta función realiza un pulso de habilitación sobre el pin EN del LCD, que normalmente está conectado al controlador HD44780 (o compatible). Recordemos cómo está definida:

```c
 void LCD_Enable(void) {
    EN = 1;
    __delay_us(5);
    EN = 0;
    __delay_us(100);
```
**¿Qué hace?**

- `EN = 1:` Activa el pin de habilitación.

- Espera 5 µs.

- `EN = 0:` Desactiva el pin, lo que provoca que el LCD lea los datos presentes en D4–D7.

- Espera otros 100 µs para que el LCD comience a procesar esos datos.

Este pulso es esencial para que el LCD interprete correctamente lo que hay en el bus de datos. Sin este pulso, no "lee" nada.

**`__delay_us(150);`**

- Después de enviar el pulso, se espera 150 microsegundos.

- Esta espera permite que el LCD tenga suficiente tiempo para procesar el comando que acaba de recibir.

>Aunque comandos rápidos pueden requerir menos, este valor está dentro del rango seguro para evitar errores de sincronización. Algunos controladores de LCD más antiguos pueden requerir estos tiempos para garantizar la correcta recepción de comandos.

```c
    LCD_Enable();

    D4 = 0; D5 = 1; D6 = 0; D7 = 0;  // Comando 0x20 >> 4 (modo 4 bits)
    LCD_Enable();

    LCD_Command(0x28);  // LCD 4 bits, 2 líneas, 5x8
    LCD_Command(0x0C);  // Display ON, cursor OFF
    LCD_Command(0x06);  // Incremento automático del cursor
    LCD_Clear();
```
**`LCD_Enable();`**

- Esto es un tercer pulso del comando 0x30 (enviado previamente como D4 = 0; D5 = 0; D6 = 1; D7 = 0).

- Forma parte de la secuencia de arranque obligatoria del LCD en 8 bits, aunque más adelante usaremos 4 bits.

```c
 D4 = 0; D5 = 1; D6 = 0; D7 = 0;  // Comando 0x20 >> 4 (modo 4 bits)
LCD_Enable();
```

- Aquí se cambia el valor de los pines de datos a 0x20 >> 4 = 0x02, o sea: 0 0 1 0.

**¿Qué comando es este?**

-Es el comando para cambiar a modo de 4 bits.

-Esto es el momento clave de la inicialización: ahora el LCD interpretará comandos usando solo los 4 bits superiores (D7–D4), enviando los 8 bits en dos tandas de 4 bits.

- Luego, se activa el LCD_Enable() para que el LCD lea ese valor y lo ejecute.


`LCD_Command(0x28);  // LCD 4 bits, 2 líneas, 5x8`

- Ahora sí se puede usar la función LCD_Command() porque el LCD ya está en modo de 4 bits.

- 0x28 = 0b00101000

Significa:

- Modo de 4 bits

- 2 líneas

- Caracteres de 5x8 píxeles

**`LCD_Command(0x0C);  // Display ON, cursor OFF`**

- 0x0C = 0b00001100

Significa:

- Display encendido (D = 1)

- Cursor apagado (C = 0)

- Cursor no parpadea (B = 0)

**`LCD_Command(0x06);  // Incremento automático del cursor`**

- **0x06 = 0b00000110**

Significa:

- El cursor se mueve a la derecha después de escribir un carácter.

- No hay desplazamiento de pantalla.

**`LCD_Clear();`**

- Envía el comando 0x01, que borra todo el contenido del LCD y coloca el cursor en la posición inicial.

- Internamente llama a:

```c
LCD_Command(0x01);
__delay_ms(2);  // Porque este comando es más lento
```

**`void LCD_String(const char *str)`**

```c
    while (*str) {
        LCD_Char(*str++);
    }
```

- Esta función recibe como parámetro un puntero a una cadena de caracteres (const char *str).
- Su propósito es mostrar una cadena de texto completa en el LCD, carácter por carácter.

**Inicio del ciclo while:**

```c
while (*str)
```
- `*str` evalúa el contenido del puntero, es decir, el carácter actual de la cadena.

- En C, las cadenas terminan con un carácter nulo ('\0', valor 0), por lo tanto:

  - Mientras el carácter no sea '\0', continúa el ciclo.

  - Cuando llega al final de la cadena, el ciclo se detiene.

**Cuerpo del ciclo:**

```c
LCD_Char(*str++);
```
- `*str` accede al carácter actual de la cadena.

- `str++` avanza el puntero al siguiente carácter.

Así, en cada iteración:

  - Se muestra el carácter actual con LCD_Char().

  - Luego se avanza al siguiente carácter para la próxima iteración.

### **Posicionamiento del cursor en el LCD**
```c
void LCD_SetCursor(unsigned char row, unsigned char col) {
    if (row == 1)
        LCD_Command(0x80 + col);
    else
        LCD_Command(0xC0 + col);
}
```
**Definición de la función:**

```c
void LCD_SetCursor(unsigned char row, unsigned char col)
```
- Esta función permite posicionar el cursor del LCD en una posición específica, dada por:

  - `row:` la fila (1 o 2).

  - `col:` la columna (0 a 15 para LCDs de 16 columnas, 0 a 19 para 20 columnas).

**Primera línea del cuerpo:**

```c
if (row == 1)
    LCD_Command(0x80 + col);
```
- Si se selecciona la primera fila (row == 1), entonces el comando de posición de cursor es:

  - `0x80` es la dirección base del inicio de la primera fila del DDRAM del LCD.

  - Se le suma col para desplazarse hacia la derecha.

**Ejemplos:**

- `LCD_SetCursor(1, 0)` → LCD_Command(0x80) → columna 0, fila 1

- `LCD_SetCursor(1, 5)` → LCD_Command(0x85) → columna 5, fila 1

**Else para la segunda fila:**

```c
else
    LCD_Command(0xC0 + col);
```
- Si la fila no es 1, se asume que es la segunda fila.

- `0xC0` es la dirección base del inicio de la segunda fila del LCD.

Ejemplos:

- `LCD_SetCursor(2, 0)` → LCD_Command(0xC0) → columna 0, fila 2

- `LCD_SetCursor(2, 10)` → LCD_Command(0xCA) → columna 10, fila 2

### **Creación de caracteres personalizados en la memoria CGRAM del LCD**

```c
void LCD_CREATECHAR(unsigned char location, unsigned char *charMap) {
    location &= 0x07;
    LCD_Command(0x40 + (location * 8));
    for (int i = 0; i < 8; i++) {
        LCD_Char(charMap[i]);
    }
}
```
Esta función define un carácter personalizado.

- `location:` número del 0 al 7 que indica en qué posición de la CGRAM se almacenará el carácter (se pueden crear hasta 8).

- `charMap:` puntero a un arreglo de 8 bytes que describe el patrón del carácter (cada byte representa una fila de 5 bits del carácter).

```c
    location &= 0x07;
```
Asegura que la posición esté entre 0 y 7. El operador &= 0x07 fuerza los 3 bits menos significativos del location (0b00000111), descartando cualquier valor fuera del rango válido para la CGRAM.

**Comando para establecer la dirección base en CGRAM donde se almacenará el nuevo carácter.**

```c
    LCD_Command(0x40 + (location * 8));
```

- `0x40` es el comando base para acceder a CGRAM.

- Se multiplica location * 8 porque cada carácter personalizado ocupa 8 direcciones de memoria (una por cada fila de píxeles del carácter).

**Escribe fila por fila el carácter en la CGRAM.**
```c
    for (int i = 0; i < 8; i++) {
        LCD_Char(charMap[i]);
    }
```
- El ciclo recorre los 8 bytes del arreglo charMap.

- Cada charMap[i] representa una fila del patrón del carácter (solo los 5 bits menos significativos son relevantes para el LCD).

## **Archivo `lcd.h`**

El archivo lcd.h es un archivo de encabezado (header file) que cumple un rol fundamental en la organización y modularización del código del proyecto. A continuación se explica su funcionamiento línea por línea y su importancia:

```c
#ifndef LCD_H
#define LCD_H
```
- Esta es una directiva de preprocesador que evita inclusiones múltiples del mismo archivo.

- Si LCD_H no ha sido definido, lo define. Así se impide que el compilador lo procese más de una vez (lo cual causaría errores).

```c
#include <xc.h>
```
- Incluye el archivo base del compilador XC8 para acceder a los registros y definiciones del PIC.

- Es necesario para manipular puertos, registros especiales y funciones como __delay_ms().

```c
#define _XTAL_FREQ 64000000
```

- Define la frecuencia del oscilador del microcontrolador (64 MHz).

- Es necesaria para que funciones como __delay_ms() y __delay_us() trabajen correctamente, ya que el compilador necesita conocer la frecuencia de reloj para calcular los retardos.

### **Definición de pines del LCD:**

```c
#define RS LATDbits.LATD5
#define EN LATDbits.LATD6
#define D4 LATCbits.LATC4
#define D5 LATCbits.LATC5
#define D6 LATCbits.LATC6
#define D7 LATCbits.LATC7
```
- Aquí se asignan nombres simbólicos a los pines de datos y control del LCD.

- Esto facilita la lectura y modificación del código, ya que puedes cambiar un pin físico simplemente modificando aquí su definición.

Ejemplo:

- RS: Registro de selección (comando/dato).

- EN: Habilitador para activar el LCD.

- D4-D7: Pines de datos (modo de 4 bits).

### **Prototipos de funciones:**

```c
void LCD_Init(void);
void LCD_Command(unsigned char cmd);
void LCD_Char(unsigned char data);
void LCD_String(const char *str);
void LCD_SetCursor(unsigned char row, unsigned char col);
void LCD_Clear(void);
void LCD_CREATECHAR(unsigned char location, unsigned char *charMap);
void LCD_Enable(void);
```

- Estas son las declaraciones (prototipos) de las funciones implementadas en lcd.c.

- Permiten que cualquier otro archivo .c que incluya este header pueda llamar a esas funciones sin errores.

```c
#endif  // LCD_H
```
Cierra la directiva de inclusión condicional iniciada al comienzo (#ifndef LCD_H).

**ENLACE A CODIGOS `newmain`, `control,c` y `control.h`.**

| Archivo          | Descripción           | Tipo       |
|------------------|-----------------------|------------|
| [newmain](/CODIGOS/newmain.c)  | Archivo Principal | C        |
| [control.h](/CODIGOS/control.h)   | Archico encabezado    | h     |
| [control.c](/CODIGOS/control.c)   | Declaración de variables     | C     |

**ENLACE A CÓDIGOS `lcd.c` y `lcd.h`**

| Archivo                         | Descripción                         | Tipo |
|---------------------------------|-------------------------------------|------|
| [lcd.h](/CODIGOS/lcd.h)         | Archivo de encabezado del LCD       | `.h` |
| [lcd.c](/CODIGOS/lcd.c)         | Implementación de funciones del LCD | `.c` |

# Diagramas

## Diagrama de conexiones

![DIAGRAMA](/IMAGENES/DIAGRAMA_CARRO.jpg)

### **Conexiones del Sensor TCRT5000 al PIC18F45K22**

| Sensor TCRT5000 | Descripción                | Pin en el PIC18F45K22 | Modo de uso   |
|------------------|----------------------------|------------------------|----------------|
| D0 (Izquierdo)   | Salida digital (comparador) | RB0                   | Entrada digital (LS) |
| D0 (Derecho)     | Salida digital (comparador) | RB1                   | Entrada digital (RS) |
| A0 (Izquierdo)   | Salida analógica proporcional | RA0 (opcional)     | Entrada analógica (ADC) |
| A0 (Derecho)     | Salida analógica proporcional | RA1 (opcional)     | Entrada analógica (ADC) |
| VCC              | Alimentación (+5V)         | +5V                    | —              |
| GND              | Tierra                     | GND                    | —              |

### **Conexiones de la Pantalla LCD 16x2 (modo 4 bits)**

| LCD 16x2       | Descripción               | Pin en el PIC18F45K22 | Notas                   |
|----------------|---------------------------|------------------------|--------------------------|
| RS             | Registro de control       | RD5                   | Selección de instrucción o datos |
| EN             | Enable                    | RD6                   | Activa la lectura/escritura |
| D4             | Datos bit 4               | RC4                   | Modo 4 bits              |
| D5             | Datos bit 5               | RC5                   | Modo 4 bits              |
| D6             | Datos bit 6               | RC6                   | Modo 4 bits              |
| D7             | Datos bit 7               | RC7                   | Modo 4 bits              |
| VSS            | Tierra                    | GND                   | —                        |
| VDD            | Alimentación              | +5V                   | —                        |
| V0             | Contraste                 | Potenciómetro         | Control de contraste     |
| RW             | Lectura/Escritura         | GND                   | Siempre escritura (RW = 0) |
| A (LED+)       | Luz de fondo positiva     | +5V (via resistencia) | Iluminación              |
| K (LED−)       | Luz de fondo negativa     | GND                   | Iluminación              |

### **Conexiones del Módulo L298N al PIC18F45K22**

| Módulo L298N  | Descripción                | Pin en el PIC18F45K22 | Notas                   |
|----------------|----------------------------|------------------------|--------------------------|
| IN1            | Motor A control 1          | RD0                   | Definido como IN1        |
| IN2            | Motor A control 2          | RD1                   | Definido como IN2        |
| IN3            | Motor B control 1          | RD2                   | Definido como IN3        |
| IN4            | Motor B control 2          | RD3                   | Definido como IN4        |
| ENA            | PWM Motor A                | RC2 (CCP1)            | Control de velocidad     |
| ENB            | PWM Motor B                | RC1 (CCP2)            | Control de velocidad     |
| VCC            | Alimentación lógica (5V)   | +5V                   | Fuente para el L298N     |
| GND            | Tierra                     | GND                   | —                        |
| 12V (VSS)      | Alimentación de motor      | Fuente externa (12V)  | Potencia del motor       |

# Complicaciones encontradas durante el desarrollo:
Durante el diseño y construcción del prototipo, se presentaron diversas dificultades técnicas que exigieron soluciones prácticas y mejor comprensión de los componentes electrónicos involucrados. A continuación, se describen las principales complicaciones encontradas:
 
 ## 1. Sobretensión en el LCD:
Una de las primeras fallas detectadas ocurrió al conectar accidentalmente el display LCD a una fuente de 12V, cuando su operación segura requiere un máximo de 5V. Esta sobretensión causó un daño irreversible al módulo, lo que obligó a reemplazarlo por uno nuevo.

## Aprendizaje: 
Se comprendió la importancia de verificar siempre los niveles de voltaje antes de realizar conexiones, especialmente en componentes sensibles. Como medida correctiva, se etiquetaron las líneas de alimentación y se implementó el uso de conectores con código de color para minimizar el riesgo de error.
 
 ##  2. Fuente de alimentación del PIC:
Durante la fase inicial no se identificó que el módulo puente H (L298N) contaba con una salida de 5V que podía usarse para alimentar directamente el microcontrolador. Esto generó confusión y retrasos en la puesta en marcha del sistema.

## Solución:
 Para garantizar una alimentación estable y segura, se optó por incluir un regulador de voltaje LM7805, el cual convierte los 12V de la batería en 5V regulados, protegiendo así al PIC de posibles fluctuaciones.
  ## 3. Problemas con sensores TCRT5000:
Los sensores infrarrojos reflectivos TCRT5000, utilizados para detectar la línea sobre el suelo, presentaron lecturas inestables. Esta inestabilidad afectaba directamente la capacidad del robot para seguir la trayectoria correctamente, provocando movimientos erráticos o pérdida de la línea.

* Causas del problema:
* La variabilidad en la reflectividad del suelo (brillo o tipo de superficie).

* Ruido eléctrico en las señales de salida de los sensores.

* Lecturas instantáneas sin procesamiento, lo que hacía que cualquier lectura espuria provocara un comportamiento incorrecto.

## Solución implementada:
Se mejoró la lógica de lectura de los sensores incorporando un sistema de muestreo y votación. En lugar de tomar una sola lectura, se tomaron múltiples muestras en un corto periodo de tiempo. Si al menos 3 de 5 muestras coincidían, se consideraba que la línea estaba presente.

## Ejemplo de código utilizado:


```c 
unsigned char i, countL = 0, countR = 0;
for (i = 0; i < 5; i++) {
    if (LS) countL++;      // LS: sensor izquierdo
    if (RS) countR++;      // RS: sensor derecho
    __delay_us(read_time); // Pequeño retardo entre lecturas
}

unsigned char leftValue = (countL >= 3) ? 1 : 0;
unsigned char rightValue = (countR >= 3) ? 1 : 0;

```
* Este enfoque simple pero efectivo actúa como un filtro de ruido digital, donde solo se valida una detección si hay consenso en varias muestras. De esta forma, se evita que un solo pulso espurio active la lógica de seguimiento de línea.
## Resultado:
Tras aplicar esta técnica, se observó una notable mejora en la estabilidad de detección, especialmente en curvas cerradas y superficies parcialmente reflectantes. El carro respondió con mayor precisión a los cambios de dirección de la línea negra.
## 4. Robustez del código:
 Se detectó que la estructura del programa puede mejorarse con funciones modulares, mejor manejo de condiciones límite y filtrado de ruido. Se recomienda implementar rutinas de calibración y validación de señales.
 ## Aprendizaje:

 * Cada una de estas dificultades aportó significativamente al desarrollo de habilidades prácticas y teóricas, tales como:

* Diseño eléctrico seguro y confiable.

* Integración eficiente de periféricos como sensores y módulos de control.

* Buenas prácticas de programación embebida, incluyendo modularidad, manejo de errores y validación de señales.

* Diagnóstico y solución de fallos en tiempo real, clave en proyectos de robótica móvil.

## **Diagrama   PIC18F45K22**

![DIAGRAMA](/IMAGENES/DIAGRAMA%20(2).png)

### Ubicación en el diagrama:

### Internal Oscillator Block

## 1. Internal Oscillator Block + PLL
```c 
#pragma config FOSC = INTIO67  
#pragma config PLLCFG = ON
OSCCON = 0b01110000;
OSCTUNEbits.PLLEN = 1;
```

- Ubicación en el diagrama:

- Parte inferior izquierda: Internal Oscillator Block, PLL, OSCCON, OSCTUNE.

Función: Configura el sistema de reloj interno del microcontrolador a 64 MHz utilizando el oscilador interno y el PLL (Phase Locked Loop), lo que impacta directamente en la precisión de los temporizadores y PWM.

## 2. Puerto B como entrada digital
```c 
TRISBbits.TRISB0 = 1;
TRISBbits.TRISB1 = 1;
ANSELBbits.ANSB0 = 0;
ANSELBbits.ANSB1 = 0;
```
### Ubicación en el diagrama:
- Parte derecha, bloque PORTB (RB0-RB7)

Función: Configura los pines RB0 y RB1 como entradas digitales para leer sensores.

## 3 Puerto D como salida digital (control de motores)
```c 
TRISD &= ~(0x0F);
LATD &= ~(0x0F);
``` 
### Ubicación en el diagrama:

- Parte derecha, bloque PORTD (RD0-RD7)

Función: Usa RD0 a RD3 como salidas digitales para el control de motores con el puente H (por ejemplo, L298N).
## 4. PWM (CCP1 y CCP2)
```c 
TRISCbits.TRISC2 = 0;         // RC2 -> CCP1
TRISCbits.TRISC1 = 0;         // RC1 -> CCP2
CCP1CON = 0b00001100;         // Modo PWM
CCP2CON = 0b00001100;
CCPR1L = 0;
CCPR2L = 0;
```

### Ubicación en el diagrama:

- Parte inferior: Bloques ECCP1 (CCP1) y ECCP2 (CCP2)

- Asociado con PORTC (RC1, RC2)

Función: Configura los módulos de PWM usando los periféricos CCP1 y CCP2 para controlar la velocidad de los motores.
## 5. Timer2 (base de tiempo del PWM)
```c 
PR2 = 60;
T2CON = 0b00000111;
```

### Ubicación en el diagrama:

- Ubicación en el diagrama:

- Parte inferior, bloque Timer2 / Timer4 / Timer6

Función: Genera el periodo para el PWM. Timer2 controla la frecuencia del PWM usado por CCP1 y CCP2.

## 6. Memoria y lógica de control
```c 
// Uso de while(1), condiciones, funciones, etc.
```
### Ubicación en el diagrama:

- Ubicación en el diagrama:

- Parte central: bloques Instruction Decode and Control, ALU, Program Counter, etc.

Función: Se utilizan los bloques centrales del CPU para ejecución de instrucciones, operaciones lógicas, comparación y llamadas a funciones.
## 7.  LCD (manejo por GPIOs digitales)

```c 
RS = LATDbits.LATD5;
EN = LATDbits.LATD6;
D4-D7 = LATCbits.LATC4 a LATC7;
```
### Ubicación en el diagrama:

- Ubicación en el diagrama:

- PORTD y PORTC: pines usados como salida digital para comunicación con LCD (modo 4 bits).

# Conclusiones

## **Importancia del archivo lcd.c**

El archivo lcd.c es fundamental dentro de el proyecto porque contiene la implementación completa de todas las funciones necesarias para controlar una pantalla LCD alfanumérica (en modo de 4 bits), utilizando un microcontrolador PIC (como el PIC18F45K22).

**1.Encapsula el manejo del LCD**

`lcd.c` define todas las operaciones necesarias para:

- Inicializar el LCD.

- Enviar comandos.

- Mostrar caracteres y cadenas.

- Posicionar el cursor.

- Limpiar la pantalla.

- Crear caracteres personalizados (como íconos o animaciones).

> Esto permite que el resto del programa no tenga que preocuparse por los detalles de bajo nivel de cómo funciona el LCD, y simplemente llame a funciones como LCD_String() o LCD_SetCursor().

## **Importancia del archivo control.c**

El archivo control.c representa el módulo de control de movimiento del sistema basado en un microcontrolador PIC, específicamente encargado de gestionar la dirección y velocidad de dos motores DC a través del puente H L298N y señales PWM. Su diseño permite que el vehículo o robot realice movimientos como avanzar, retroceder, girar o detenerse.

### **Aspectos clave del archivo:**

**1. Modularidad y abstracción**

El código encapsula todas las acciones necesarias para el control de motores en funciones específicas (forward, turnLeft, etc.), lo que facilita su uso desde otros archivos (como main.c) sin preocuparse por los detalles de los pines o registros.

**2. Control de dirección**

Las funciones manipulan los pines IN1, IN2, IN3, IN4 para establecer la dirección de rotación de los motores conectados al L298N, permitiendo:

- **Adelante:** Motor 1 y Motor 2 giran hacia adelante.

- **Reversa (solo Motor 1):** Motor 1 gira en reversa, Motor 2 hacia adelante.

- **Giro derecha/izquierda:** Solo un motor gira, el otro está apagado, facilitando el cambio de dirección.

- Detener: Ambos motores se detienen.

**3. Control de velocidad con PWM**

Las funciones setDuty1() y setDuty2() controlan el ciclo de trabajo de los canales PWM (usando los módulos CCP1 y CCP2), que modulan la velocidad de cada motor.
Esto permite variaciones suaves de velocidad dependiendo del valor recibido (val).

**4. Configuración del PWM**

La función setupPWM() configura correctamente los periféricos CCP1 y CCP2 para generar señales PWM usando el Timer2. Se ajusta:

- Modo PWM en CCP1CON y CCP2CON.

- Pines RC1 y RC2 como salidas.

- Frecuencia de PWM usando PR2 y T2CON.

**5. Encapsula el control de bajo nivel de los motores**

  - Define cómo se configuran y activan los pines de dirección y PWM que controlan el L298N.

  - Usa funciones como setDuty1() y setDuty2() para controlar la velocidad de los motores con precisión.

  - Usa señales IN1-IN4 para manejar la dirección de los motores.

## **Conclusión Final**

El sistema presentado es un ejemplo completo y bien estructurado de un robot seguidor de línea inteligente que combina varias tecnologías: sensores digitales, control PWM, visualización en LCD y programación modular en C.

### **Fortalezas del proyecto:**

- Modularidad clara: Separación en archivos (control, lcd, main) que mejora el mantenimiento.

- Interacción Hombre-Máquina (HMI): Uso del LCD para mostrar el estado en tiempo real.

- Control preciso: Gracias al uso de PWM por hardware y lectura de sensores con filtro.

- Escalable: Fácil de mejorar, por ejemplo, agregando más sensores, Bluetooth, detección de obstáculos, etc.

### **Posibles mejoras:**

- Agregar detección de cruce de caminos o curvas cerradas.

- Implementar PID para suavizar movimientos.

- Medir el desempeño en entornos reales (velocidad, precisión, respuesta).

- Incorporar una fuente de alimentación regulada para evitar caídas de tensión al activar motores.

# IMPLEMENTACIÓN

[VIDEO DE IMPLEMENTACIÓN](https://youtube.com/shorts/qP0fclMwPqs?feature=share)

# REFERENCIAS

(S/f). *Microchip.com*. Recuperado el 12 de abril de 2025, de [https://ww1.microchip.com/downloads/en/DeviceDoc/40001412G.pdf](https://ww1.microchip.com/downloads/en/DeviceDoc/40001412G.pdf) 