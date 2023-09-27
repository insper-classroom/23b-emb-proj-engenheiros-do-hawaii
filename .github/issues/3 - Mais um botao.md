Adicionem mais um botão ao sistema, note que isso envolve modificar
o firmware para ler o botão e enviar via comunicação UART (Bluetooth)
o novo dado e também o probrama python para fazer a leitura deste novo
botão.

O protocolo atual utilizado na comunicação entre o uC e o python é o seguinte:

```
Protocolo
                Fim de pacote
            ^
            |
| Status |   'X'   |
    |
    v
    - '0': Não apertado
    - '1': Apertado
```

O protocolo envia caracteres ASCII onde cada campo é formado por 8 bits:

- `status`: 8 bits, indica o valor atual do botão
- `EOP`: 8 bits, é o fim de pacote, atualmente formado pelo caractere **X**.

**Já pense em modificar o protocolo pensando no uso final, se não você terá
que refazer essa etapa mais para frente.**

O envio deste protocolo se encontra no while da `task_bluetooth`:

```c
// envia status botão
while(!usart_is_tx_ready(USART_COM)) { TaskDelay(10 / portTICK_PERIOD_MS); }
usart_write(USART_COM, button1);

// envia fim de pacote
while(!usart_is_tx_ready(USART_COM)) { vTaskDelay(10 / portTICK_PERIOD_MS); }
usart_write(USART_COM, eof);
```

Existem várias maneiras de vocês modificarem este protocolo para enviar mais um
botão, a dupla deve decidir como fazer.

**Lembrem de adequar o python para ler o novo protocolo**

