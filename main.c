/**************************************************************
***************************************************************

UNIVERSIDADE FEDERAL DO RIO GRANDE DO NORTE (UFRN)
DEPARTAMENTO DE ENGENHARIA DE COMPUTAÇÃO E AUTOMAÇÃO (DCA)
DCA0114 - COMPUTAÇÃO GRÁFICA - T01 (2020.2 - 35M12)

PROF: DR. LUIZ MARCOS GARCIA GONCALVES

GRUPO:  ANGELO LEITE MEDEIROS DE GOES
        ARIEL DA SILVA ALSINA
        LUIZ PAULO DE CARVALHO ALVES

***************************************************************
*** ALGORITMO DE RAYCASTING USANDO OPENGL E BIBLIOTECA GLUT ***
***************************************************************

***************************************************************
***************************************************************

///***********************///
/// BIBLIOTECAS
///***********************///

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

///***********************///
/// ESTRUTURAS DE DADOS
///***********************///

struct Esfera{
    GLdouble* coords;
    GLdouble* kd;    // tamanho 3: R,G,B
    GLdouble* ks; // tamanho 3: R,G,B
    GLdouble r;
    GLdouble nshiny;
};

struct LuzPontual{
    GLdouble* coords;
    GLdouble IL;
    GLdouble fatt;
};

struct LuzAmbiente{
    GLdouble Ia;
    GLdouble ka;
};

struct IntersectCoefs{
    GLdouble delta, b;
};

///***********************///
/// DECLARAÇÕES INICIAIS
///***********************///

GLdouble* lumi_xy;      /* Luminância no pixel (x, y) */
GLdouble x_tela, y_tela;      /* Coordenadas de mundo do pixel viewport(x, y) */
GLdouble* lf;                 /* lookfrom ou 'origem' */
struct Esfera *esferas;       /* Lista de esferas na cena*/
struct LuzPontual *luzPont;   /* Lista de luzes pontuais */
struct LuzAmbiente luzAmb;
int n_esferas, n_luzPont;

///***********************///
/// CONFIGURAÇÕES GERAIS
///***********************///

GLdouble fov = 45.0;
GLdouble znear = 1.0;   /* Distância focal */
GLdouble zfar = 100.0;
GLdouble stepSize = 5.0;
GLint renderizarSombras = 1;    /* = 1 para renderizar sombras */

///***********************///
/// OPERAÇÃES COM VETORES
///***********************///

GLdouble *normalizarVetor(GLdouble* arr, GLint n){
    GLdouble* normal = (GLdouble*) malloc(sizeof(GLdouble) * n);
    GLdouble sum = 0.0;

    for(int i = 0; i < n; i++)
        sum = sum + arr[i]*arr[i];

    GLdouble sqrt_sum = sqrt(sum);

    for(int i = 0; i < n; i++)
        normal[i] = arr[i]/sqrt_sum;

    return normal;
}

GLdouble dot_product(GLdouble* v, GLdouble* u, GLint n)
{
    GLdouble result = 0.0;
    for (int i = 0; i < n; i++)
        result += v[i]*u[i];
    return result;
}

GLdouble* cnt_product(GLdouble* v, GLdouble c, GLint n)
{
    GLdouble* result = (GLdouble*) malloc(sizeof(GLdouble) * n);
    for (int i = 0; i < n; i++)
        result[i] = c * v[i];
    return result;
}

GLdouble* sub_arrays(GLdouble* v, GLdouble* u, GLint n)
{
    GLdouble* result = (GLdouble*) malloc(sizeof(GLdouble) * n);
    for (int i = 0; i < n; i++)
        result[i] = v[i] - u[i];
    return result;
}

GLdouble* add_arrays(GLdouble* v, GLdouble* u, GLint n)
{
    GLdouble* result = (GLdouble*) malloc(sizeof(GLdouble) * n);
    for (int i = 0; i < n; i++)
        result[i] = v[i] + u[i];
    return result;
}

GLdouble euclid_dist(GLdouble* v, GLdouble* u, GLint n)
{
    GLdouble result = 0;
    for (int i = 0; i < n; i++)
        result += pow((v[i] - u[i]),2);
    return sqrt(result);
}

///***********************///
/// RECEBE COORDENADAS DE TELA
/// RETORNA COORDENADAS DE MUNDO
///***********************///

GLdouble* screenToWorldCoord(int x, int y){

    GLint viewport[4];
    GLdouble mvmatrix[16], projmatrix[16];
    GLint realy;            /*  OpenGL y coordinate position  */
    GLdouble wx, wy, wz;    /*  returned world x, y, z coords  */
    GLdouble* temp = (GLdouble*) malloc(sizeof(GLdouble) * 3);

    glGetIntegerv (GL_VIEWPORT, viewport);
    glGetDoublev (GL_MODELVIEW_MATRIX, mvmatrix);
    glGetDoublev (GL_PROJECTION_MATRIX, projmatrix);

    realy = viewport[3] - (GLint) y - 1;

    /// ATUALIZANDO COORDENADAS DE MUNDO EM z = 0.0 (znear)
    gluUnProject ((GLdouble) x, (GLdouble) realy, 0.0, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);
    // Usados para desenhar o pixel em (x_tela, y_tela)
    x_tela = wx;
    y_tela = wy;

    /// COORDENADAS DE MUNDO EM z = 1.0 (zfar)
    gluUnProject ((GLdouble) x, (GLdouble) realy, 1.0, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);

    temp[0] = wx;
    temp[1] = wy;
    temp[2] = wz;

    return temp;
}

///***********************///
/// RECEBE POSIÇÃO DO OBSERVADOR, DIREÇÃO DO RAIO NORMALIZADO E ESFERA
/// RETORNA COEFICIENTES DO CALCULO DE INTERSECÇÃO DO RAIO COM UMA ESFERA
///***********************///

struct IntersectCoefs calcularIntersectEsfera(GLdouble* lookfrom, GLdouble* direcao_normal, struct Esfera esfera){

    /// DESCOMPACTANDO COORDENADAS DE ORIGEM
    GLdouble xo = lookfrom[0];
    GLdouble yo = lookfrom[1];
    GLdouble zo = lookfrom[2];

    /// DIREÇÃO NORMALIZADA
    GLdouble xd = direcao_normal[0];
    GLdouble yd = direcao_normal[1];
    GLdouble zd = direcao_normal[2];

    /// COORDENADAS DE ESFERA
    GLdouble xc = esfera.coords[0];
    GLdouble yc = esfera.coords[1];
    GLdouble zc = esfera.coords[2];
    GLdouble r = esfera.r;

    /// VALORES DO CÁLCULO DA INTERSERCÇÃO
    GLdouble delta;
    GLdouble a = 1.0;
    GLdouble b = 2 * (xd * (xo - xc) + yd * (yo - yc) + zd * (zo - zc));
    GLdouble c = (xo - xc)*(xo - xc) + (yo - yc)*(yo - yc) + (zo - zc)*(zo - zc) - r*r;

    delta = b*b - 4*a*c;

    struct IntersectCoefs coefs = {delta, b};

    return coefs;
}

///***********************///
/// CHECA POR INTERSECÇÃO DO RAIO CASTADO ATRAVES DO PIXEL EM (x,y) COM OBJETOS
/// CALCULA E RETORNA ILUMINAÇÃO TOTAL NAQUELE PIXEL, OU -1.0, CASO NÃO HAJA INTERSECÇÃO
///***********************///

GLdouble* calcularIluminacao(int x, int y, GLdouble* lookfrom){

    /// OBTENDO COORDENADAS DO VETOR DE DIREÇÃO NORMALIZADO
    GLdouble* direcao = screenToWorldCoord(x, y);
    GLdouble* direcao_normal = normalizarVetor(direcao, 3);

    GLdouble closest_intersection = zfar;
    int sphr_i = 0;

    for (int i = 0; i < n_esferas; i++){

        // Calcula o valor de 'delta' e 'b' da intersecção raio-esfera
        struct IntersectCoefs coefs = calcularIntersectEsfera(lookfrom, direcao_normal, esferas[i]);

        // Se (delta > 0) existe raízes para equação, logo, há intersecção
        if(coefs.delta >= 0){

            GLdouble t1, t2, t;
            t1 = (-coefs.b + sqrt(coefs.delta))/(2*1.0);
            t2 = (-coefs.b - sqrt(coefs.delta))/(2*1.0);

            // Escolhe a menor raiz (mais próxima do observador)
            t = (fabs(t1) < fabs(t2)) ? t1 : t2;

            // Verifica se essa raiz é a mais próxima até então, se for armazena ela
            if (fabs(t) < fabs(closest_intersection)){
                closest_intersection = t;
                sphr_i = i;
            }
        }
    }

    // Caso haja intersecção dentro do frustum
    if (fabs(closest_intersection) < zfar){
        GLdouble ambiente;
        ambiente = luzAmb.Ia * luzAmb.ka;

        /// Três canais: R, G e B
        GLdouble* I = (GLdouble*) malloc(sizeof(GLdouble) * 3);
        I[0] = ambiente;
        I[1] = ambiente;
        I[2] = ambiente;

        /// Cores da esfera mais à frente (a que representa o pixel)
        GLdouble kdr = esferas[sphr_i].kd[0];
        GLdouble kdg = esferas[sphr_i].kd[1];
        GLdouble kdb = esferas[sphr_i].kd[2];

        GLdouble ksr = esferas[sphr_i].ks[0];
        GLdouble ksg = esferas[sphr_i].ks[1];
        GLdouble ksb = esferas[sphr_i].ks[2];

        GLdouble ilumAcumuladaLuzPont = 0.0;

        /// VETOR SUPERFICIE (S) = lookfrom + distance * direção_normalizada
        GLdouble* superficie = add_arrays(lookfrom, cnt_product(direcao_normal, closest_intersection, 3), 3);

        /// VETOR NORMAL (N) = S - Esfera
        GLdouble* normal = normalizarVetor(sub_arrays(superficie, esferas[sphr_i].coords, 3), 3);

        /// VETOR OBSERVADOR (O) = lookfrom - S
        GLdouble* observador = normalizarVetor(sub_arrays(lookfrom, superficie, 3), 3);


        for (int i = 0; i < n_luzPont; i++){


            /// VETOR LUZ (L) = Luz - S
            GLdouble* luz = normalizarVetor(sub_arrays(luzPont[i].coords, superficie, 3), 3);

            /// PRODUTO NOTÁVEL LUZ . NORMAL
            GLdouble LN = dot_product(luz, normal, 3);

            // Checa se deve ou não renderizar sombras
            if(renderizarSombras == 1){
                // Checa se há sombra na direção da luz
                // Caso haja retorne apenas a iluminação ambiente
                GLint haSombra = -1;     //Flag 'haSombra'
                for(int j = 0; j < n_esferas; j++){
                    struct IntersectCoefs shadow_coefs = calcularIntersectEsfera(superficie, luz, esferas[j]);
                    if(shadow_coefs.delta >= 0 && sphr_i != j){

                        GLdouble distObjetoLuz = euclid_dist(esferas[sphr_i].coords, luzPont[i].coords, 3);
                        GLdouble distObstaculoLuz = euclid_dist(esferas[j].coords, luzPont[i].coords, 3);

                        // Checa se a distancia euclidiana obstaculo-luz é menor que objeto atual-luz
                        // Só aplica sombra se o obstaculo estiver entre objeto e luz, ou seja, está entre eles
                        // E também se o vetor normal a superficie e luz apontarem no mesmo sentido

                        if(distObstaculoLuz < distObjetoLuz && LN > 0)
                        {
                            I[0] = ambiente;
                            I[1] = ambiente;
                            I[2] = ambiente;

                            haSombra = 1;
                            break;
                        }
                    }
                }
                if(haSombra == 1)
                    break;
            }

            /// VETOR REFLETIDO (R) = 2N(N.L) - L
            GLdouble* refletido = sub_arrays(
                                    cnt_product(
                                    cnt_product(normal,
                                    dot_product(normal, luz, 3), 3), 2, 3), luz, 3);

            /// PRODUTO NOTÁVEL REFLETIDO . OBSERVADOR
            GLdouble RO = dot_product(observador, refletido, 3);

            /// CÁLCULO DA ILUMINAÇÃO TOTAL

            GLdouble* difusa = (GLdouble*) malloc(sizeof(GLdouble) * 3);
            difusa[0] = kdr * LN;
            difusa[1] = kdg * LN;
            difusa[2] = kdb * LN;

            GLdouble* especular = (GLdouble*) malloc(sizeof(GLdouble) * 3);
            especular[0] = 0;
            especular[1] = 0;
            especular[2] = 0;

            if(RO > 0){
                especular[0] = ksr * pow(RO, esferas[sphr_i].nshiny);
                especular[1] = ksg * pow(RO, esferas[sphr_i].nshiny);
                especular[2] = ksb * pow(RO, esferas[sphr_i].nshiny);
            }

            I[0] += luzPont[i].fatt * luzPont[i].IL * (difusa[0] + especular[0]);
            I[1] += luzPont[i].fatt * luzPont[i].IL * (difusa[1] + especular[1]);
            I[2] += luzPont[i].fatt * luzPont[i].IL * (difusa[2] + especular[2]);
        }


        /// REDUZIR PARA VALOR [0, 1] e depois multiplica pelo fator da cor
        I[0] = (I[0] / (2000));
        I[1] = (I[1] / (2000));
        I[2] = (I[2] / (2000));

        return I;
    } else {
        GLdouble* I = (GLdouble*) malloc(sizeof(GLdouble) * 3);
        I[0] = -1.0;
        I[1] = -1.0;
        I[2] = -1.0;

        return I;
    }
}

///***********************///
/// AJUSTA E REDESENHA O FRAME ATUAL
/// CASO A JANELA MUDE DE PROPORÇÃO
///***********************///

void reshape (int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity();
    gluPerspective (fov, (GLfloat) w/(GLfloat) h, znear, zfar);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt (0.0, 0.0, znear, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);
}

///***********************///
/// FUNÇÃO QUE DESENHA PIXELS NA TELA
/// SOMENTE SE A LUMINANCIA DO PIXEL FOR POSITIVA
/// FAZ O DISPLAY DO FRAME ATUAL DA TELA CALCULANDO ILUMINA��O
/// E ATUALIZANDO display() PARA CADA PIXEL DEFINIDO PELO viewport
///***********************///

void display(void){

    GLint viewport[4];
    glGetIntegerv (GL_VIEWPORT, viewport);
    reshape(viewport[2], viewport[3]);
    glClear(GL_COLOR_BUFFER_BIT);

    int i, j;
    for (i = 0; i <= viewport[2]; i++){         /* viewport[2] = comprimento da tela em pixels */
        for (j = 0; j <= viewport[3]; j++){     /* viewport[3] = altura da tela em pixels */
            lumi_xy = calcularIluminacao(i, j, lf);

            if(lumi_xy[0] > 0){
                glBegin(GL_POINTS);
                    glColor3d(lumi_xy[0], lumi_xy[1], lumi_xy[2]);   /* Ajusta cor do pixel em escala cinza */
                    glVertex2d(x_tela, y_tela);             /* Desenha um ponto na cor definida em (x_tela, y_tela)*/
                glEnd();

                /* Reseta luminancia para -1.0*/
                lumi_xy[0] = -1.0;
                lumi_xy[1] = -1.0;
                lumi_xy[2] = -1.0;
            }
        }
        // Somente liberamos o buffer quando TODOS os pixeis forem calculados.
        glFlush ();
    }

    printf("Terminou display\n");
}

///***********************///
/// CONSTRUTOR DE ESFERAS
///***********************///

struct Esfera constructSphere(GLdouble x, GLdouble y, GLdouble z, GLdouble kdr, GLdouble kdg, GLdouble kdb, GLdouble ksr, GLdouble ksg, GLdouble ksb, GLdouble r, GLdouble nshiny){
    GLdouble* coords_esf = (GLdouble*) malloc(sizeof(GLdouble) * 3);
    coords_esf[0] = x;
    coords_esf[1] = y;
    coords_esf[2] = z;

	GLdouble* kd = (GLdouble*) malloc(sizeof(GLdouble) * 3);
	kd[0] = kdr;
	kd[1] = kdg;
	kd[2] = kdb;

    GLdouble* ks = (GLdouble*) malloc(sizeof(GLdouble) * 3);
    ks[0] = ksr;
	ks[1] = ksg;
	ks[2] = ksb;

    struct Esfera e = {coords_esf, kd, ks, r, nshiny};

    return e;
}

///***********************///
/// CONSTRUTOR DE LUZ PONTUAL
///***********************///


struct LuzPontual constructLight(GLdouble x, GLdouble y, GLdouble z, GLdouble IL_lp, GLdouble fatt){
    GLdouble* coords_lp = (GLdouble*) malloc(sizeof(GLdouble) * 3);
    coords_lp[0] = x;
    coords_lp[1] = y;
    coords_lp[2] = z;

    struct LuzPontual lp = {coords_lp, IL_lp, fatt};

    return lp;
}

///***********************///
/// INICIALIZAÇÃO GERAL
/// CHAMADO UMA ÚNICA VEZ
///***********************///

void init(void)
{
    // Especificando qual cor é que glClear(GL_COLOR_BUFFER_BIT) vai utilizar
    glClearColor (0.0, 0.0, 0.0, 0.0);

    /// INICIALIZANDO ESFERAS
    // params: constructSphere(GLdouble x, GLdouble y, GLdouble z, GLdouble kdr, GLdouble kdg, GLdouble kdb,
    //                         GLdouble ksr, GLdouble ksg, GLdouble ksb, GLdouble r, GLdouble nshiny)

    /// CENA 1

    n_esferas = 3;
    esferas = (struct Esfera*) malloc(sizeof(*esferas) * n_esferas);
    esferas[0] = constructSphere(0.0, 10.0, -60.0,   1.0,0.0,0.0,   1.0,1.0,1.0,  4.0, 10);     // Vermelho
    esferas[1] = constructSphere(-10.0, 0.0, -50.0,  0.0,1.0,0.0,   1.0,1.0,1.0,  8.0, 30);     // Verde
    esferas[2] = constructSphere(10.0, 10.0, -40.0,   0.0,0.0,1.0,   1.0,1.0,1.0,  5.0, 50);     // Azul


    /// CENA 2
    /*
    n_esferas = 4;
    esferas = (struct Esfera*) malloc(sizeof(*esferas) * n_esferas);
    esferas[0] = constructSphere(5.0, 5.0, -30.0,   1.0,0.0,0.0,   1.0,1.0,1.0,  3.0, 10);     // Vermelho
    esferas[1] = constructSphere(-5.0, 5.0, -20.0,  0.0,1.0,0.0,   1.0,1.0,1.0,  3.0, 30);     // Verde
    esferas[2] = constructSphere(-5.0, -5.0, -50.0,   0.0,0.0,1.0,   1.0,1.0,1.0,  3.0, 50);   // Azul
    esferas[3] = constructSphere(-3.0, 3.0, -30.0,  1.0,1.0,0.0,   1.0,1.0,1.0,  5.0, 80);     // Amarelo
    */

    /// CENA 3 - Ilusão de perspectiva
    /*
    n_esferas = 6;
    esferas = (struct Esfera*) malloc(sizeof(*esferas) * n_esferas);
    esferas[0] = constructSphere(-4.0, 8.0, -30.0,   1.0,0.0,0.0,   1.0,1.0,1.0,  3.0, 30);     // Vermelho
    esferas[1] = constructSphere(4.0, 8.0, -30.0,  0.0,1.0,0.0,   1.0,0.0,0.0,  3.0, 30);       // Verde
    esferas[2] = constructSphere(0.0, 0.0, -60.0,   0.0,0.0,1.0,   1.0,1.0,1.0,  5.0, 30);     // Azul
    esferas[3] = constructSphere(-8.0, 0.0, -30.0,  1.0,1.0,0.0,   1.0,1.0,1.0,  3.0, 30);     // Amarelo
    esferas[4] = constructSphere(8.0, 0.0, -30.0,  1.0,0.0,1.0,   1.0,1.0,1.0,  3.0, 30);      // Rosa
    esferas[5] = constructSphere(0.0, -8.0, -30.0,   0.0,1.0,1.0,   1.0,1.0,1.0,  3.0, 30);     // Ciano
    */

    /// INICIALIZANDO LUZ AMBIENTE
    luzAmb.Ia = 300;
    luzAmb.ka = 0.8;

    /// INICIALIZANDO LUZES PONTUAIS
    n_luzPont = 3;

    luzPont = (struct LuzPontual*) malloc(sizeof(*luzPont) * n_luzPont);

    luzPont[0] = constructLight(-20.0, -10.0, -10.0, 1800, 1);
    luzPont[1] = constructLight(20.0, -20.0, -10.0, 600, 1);
    luzPont[2] = constructLight(0.0, 40.0, -10.0, 300, 1);

    /// INICIALIZANDO LOOKFROM
    lf = (GLdouble*) malloc(sizeof(GLdouble) * 3);
    lf[0] = 0.0;
    lf[1] = 0.0;
    lf[2] = 10.0;

    /// INICIALIZANDO O VETOR DE LUMINÂNCIAS
    lumi_xy = (GLdouble*) malloc(sizeof(GLdouble) * 3);
    lumi_xy[0] = -1.0;
    lumi_xy[1] = -1.0;
    lumi_xy[2] = -1.0;

    /// DEFININDO TAMANHO DO PIXEL DE DESENHO
    glPointSize(1);
}

///***********************************///
/// COMANDOS DE MOVIMENTO VIA TECLADO
///***********************************///

void keyboard (unsigned char key, int x, int y)
{
    switch (key) {
        case 'n':
            display();
            break;
        case 'w':
            lf[2] -= stepSize;
            display();
            break;
        case 'a':
            lf[0] += stepSize;
            display();
            break;
        case 's':
            lf[2] += stepSize;
            display();
            break;
        case 'd':
            lf[0] -= stepSize;
            display();
            break;
        case 'q':
            lf[1] -= stepSize;
            display();
            break;
        case 'e':
            lf[1] += stepSize;
            display();
            break;

        default:
            break;
    }
}

///***********************///
/// MAIN
///***********************///

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize (500, 500);
    glutInitWindowPosition (100, 100);
    glutCreateWindow (argv[0]);
    init ();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
