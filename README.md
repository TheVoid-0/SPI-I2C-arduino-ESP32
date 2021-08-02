# SPI-I2C-arduino-ESP32
Projeto teste para implementação dos protocolos I2C e SPI entre microcontroladores (ArduinoUno&amp;ESP32)

<h1> Branch para simulação no proteus! </h1>

<br>
<h4>NOTE:</h4>
<p>
O código apresenta instabilidades nas interrupções realizadas pelo SPI, o arduino escravo (direita no simulador proteus) congela/interrompe o funcionamento após um determinado
número de interrupções realizadas, o curioso é que o número de interrupções necessárias varia de acordo com linhas de código que não deveriam ser relevantes para o funcionamento
do programa, por exemplo, se deixarmos uma linha de código <b>lcd.print("teste");</b> no loop principal antes da função slaveSPI(), o número de interrupções necessárias até
o travamento é <b>4</b>, porém se essa linha de código for retirada, deixando no loop principal apenas a função slaveSPI(), então o número de interrupções necessárias sobe para
13, o resultado continua variando dependendo do que é inserido no loop principal, mesmo que não tenha relação nenhuma com o funcionamento da ISR do SPI.
<p/>
<br>
<h4>NOTE_2:</h4>
<p>
Apesar desse código funcionar no simulador Proteus (salvo com a instabilidade escrita acima), o arduino físico apresenta um comportamento completamente diferente, enquanto
no Proteus é possível transferir Structs de vários bytes (testados até 30bytes), o arduino físico apresenta inconsistências no recebimento das informações através das interrupções,
muitas vezes a interrupção não é disparada, e nos da a sensação de que o código "está lento" tendo em vista de que até prints na saída serial as vezes são deixados pela metade
(sem completar a frase) e após retornar da interrupção o comportamento continua estranho
<p/>
