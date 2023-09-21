> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

Vamos agora criar um sistema de emergência que aciona um alarme se o valor da distância passar de 2m.

Para isso iremos criar uma nova `task_alarm` que deve ser acionada por um semáforo e ela irá reproduzir um som (tenebroso) enquanto o valor da disância tiver maior que 2m. 

1. `task_oled` Libera semáforo para a `task_alarm` soar o alarme, quando a distância for maior que 2m.

Agora pense em como fazer para a `task_oled` desligar o alame se a distância dimininuir de 2m.
