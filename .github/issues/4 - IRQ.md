A implementação atual sempre envia o valor do botão, mesmo se ele continua
o mesmo. E isso não é bom, pois sobrecarrega a comunicação e adiciona um delay
no envio dos dados que importam.

Modifique o firmware para apenas enviar pelo protocolo quando o botão muda de valor
(apertado --> aberto, aberto --> apertado).

**É recomendando que implemente interrupção (e semáforo) para implementar isso.**

Taréfa:

- Implemente IRQ e callback nos botões
    - Você vai precisar enviar quando o botão foi apertado e solto!
    - Uma `queue` é o mais indicado aqui! 
    - Você pode definir que esta `queue` recebe uma `struct` onde você pode definir o `id` do botão e o `status` dele.
- Comunique o callback dos botões via `queue` ou `xsemaphore` com a `task_bluetooth`
- Agora só envie para o bluetooth quando você tiver um novo valor na `queue`.
