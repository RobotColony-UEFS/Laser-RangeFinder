# Laser Range Finder on Raspberry Pi

Este trabalho apresenta um sistema de detecção de laser de linha a partir da utilização de técnicas de processamento de imagem, para determinar a distância à múltiplos distância obstáculos em um ambiente robótico. Os métodos propostos apresentam taxa de erro de apriximadamente 1,36% e tempo de resposta de 153 ms, quando executados na plataforma computacional Raspberry Pi B+.

## Trabalhos Publicados

**Sistema de medição de distância baseado em visão computacional utilizando laser de linha**. Walber C. de Jesus Rocha; Samuel Rebouças de Jesus; João Carlos N. Bittencourt. *In: Escola Regional Bahia-Alagoas-Sergipe 2019*, Ilhéus.

[[BibTeX](https://scholar.googleusercontent.com/scholar.bib?q=info:Pu17r4RQsmYJ:scholar.google.com/&output=citation&scisdr=CgUwocpBEKvDsoUVYTw:AAGBfm0AAAAAXkgQeTwFILuZ1FhZN2JXVktfQDYpBxHi&scisig=AAGBfm0AAAAAXkgQebfeddtHOx-TRHvuq8geXOherPSt&scisf=4&ct=citation&cd=-1&hl=pt-BR)] [[PDF](https://sol.sbc.org.br/index.php/erbase/article/view/8997/8898)] 

**Desenvolvimento de um Sistemas de Medição de Distância Baseado em Visão Computacional Utilizando Laser de Linha**. Walber Conceição de Jesus Rocha. *Trabalho de Conclusão de Curso, Bacharelado em Ciências Exatas e Tecnológicas, UFRB*. 2018.

[[PDF](https://www2.ufrb.edu.br/bcet/components/com_chronoforms5/chronoforms/uploads/tcc/20190604191229_2018.2_TCC_Walber_Conceio_de_Jesus_Rocha_-_Desenvolvimento_de_um_sistema_de_medio_de_distncia_baseado_em_viso_computacional_utilizando_laser_de_linha.pdf)]

**Sistema de Medição de Distância Baseado em Visao Computacional Utilizando Laser de Linha**.  Walber C. de Jesus Rocha; Samuel Rebouças de Jesus; João Carlos N. Bittencourt. *In.:* Revista de Sistemas e Computação - RSC (*invited paper*). 2019.

[[BibTeX](https://scholar.googleusercontent.com/scholar.bib?q=info:XpS-wCYma3cJ:scholar.google.com/&output=citation&scisdr=CgUwoco8EKvDs-S_3Lo:AAGBfm0AAAAAXym6xLoecyIg-9Ffzak45z6vH5MRF46-&scisig=AAGBfm0AAAAAXym6xCn1HL6gThIpBYnAIFwPcGpA8QmK&scisf=4&ct=citation&cd=-1&hl=pt-BR)] [[PDF](https://revistas.unifacs.br/index.php/rsc/article/view/6643)]

### Guia para Execução do Projeto

#### Instalação dos módulos
```
pip3 install math, cv2, numpy
```

#### Executar o projeto
```
python3 laser_rangefinder.py

```
#### Escolha entre os métodos de detecção do laser de linha
```
Agregação de pixels, utilizar a função: pixel_aggregation('Caminho da imagem')
Gradiente de Sobel, utilizar a função: gradient_sobel('Caminho da imagem')
```

#### Para diferentes distâncias, alterar o caminho da imagem de entrada
```
img = cv2.imread('Caminho da imagem')
python3 laser_rangefinder.py

```

### Execução do projeto na plataforma computacional Raspberry PI

#### Instalação dos módulos
```
pip install picamera, math, cv2, numpy, smbus, gpio
```
#### Escolha entre os métodos de detecção do laser de linha
```
Agregação de pixels, utilizar a função: pixel_aggregation('Caminho da imagem')
Gradiente de Sobel, utilizar a função: gradient_sobel('Caminho da imagem')
```

#### Executar projeto
```
rangefinder-rpi/
python rangefinder.py
```
