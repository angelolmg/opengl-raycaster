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

TODO:
 - CÁLCULO DA ILUMINAÇÃO PARA OS 3 CANAIS DE CORES
 - (TALVEZ) CÁLCULO DINÂMICO DE fatt, POR fatt = 1/d^2, SENDO d A DISTÂNCIA ENTRE A LUZ FOCAL E A SUPERÍCIE

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
    GLdouble* cor_esfera;    // tamanho 3: R,G,B
    GLdouble r;
    GLdouble kd;
    GLdouble ks;
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

///***********************///
/// DECLARAÇÕES INICIAIS
///***********************///

GLdouble lumi_xy = -1.0;      /* Luminância no pixel (x, y) */
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
GLdouble stepSize = 10.0;

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
/// CHECA POR INTERSECÇÃO DO RAIO CASTADO ATRAVES DO PIXEL EM (x,y) COM OBJETOS
/// CALCULA E RETORNA ILUMINAÇÃO TOTAL NAQUELE PIXEL, OU -1.0, CASO NÃO HAJA INTERSECÇÃO
///***********************///

GLdouble calcularIluminacao(int x, int y, GLdouble* lookfrom, int n_esferas, int n_luzes){

    /// DESCOMPACTANDO COORDENADAS DE ORIGEM E DE ESFERA
    GLdouble xo = lookfrom[0];
    GLdouble yo = lookfrom[1];
    GLdouble zo = lookfrom[2];

    /// OBTENDO COORDENADAS DO VETOR DE DIREÇÃO NORMALIZADO
    GLdouble* direcao = screenToWorldCoord(x, y);
    GLdouble* direcao_normal = normalizarVetor(direcao, 3);

    GLdouble xd = direcao_normal[0];
    GLdouble yd = direcao_normal[1];
    GLdouble zd = direcao_normal[2];

    double closest_intersection = zfar;
    int sphr_i = 0;
    // printf("closest intersect: %lf \n", closest_intersection);
    for (int i = 0; i < n_esferas; i++){
        GLdouble xc = esferas[i].coords[0];
        GLdouble yc = esferas[i].coords[1];
        GLdouble zc = esferas[i].coords[2];

        GLdouble r = esferas[i].r;

        /// VALORES DO CÁLCULO DA INTERSERCÇÃO
        double delta;
        double a = 1.0;
        double b = 2 * (xd * (xo - xc) + yd * (yo - yc) + zd * (zo - zc));
        double c = (xo - xc)*(xo - xc) + (yo - yc)*(yo - yc) + (zo - zc)*(zo - zc) - r*r;

        delta = b*b - 4*a*c;

        // Se (delta > 0) existe raízes para equação, logo, há intersecção
        if(delta >= 0){

            double t1, t2, t;
            t1 = (-b + sqrt(delta))/(2*a);
            t2 = (-b - sqrt(delta))/(2*a);

            // Escolhe a menor raiz (mais próxima do observador)
            t = (fabs(t1) < fabs(t2)) ? t1 : t2;
            // Verifica se essa raiz é a mais próxima até então, se for armazena ela
            if (fabs(t) < fabs(closest_intersection)){
                closest_intersection = t;
                sphr_i = i;
            }
        }
    }

    if (fabs(closest_intersection) < zfar){
        GLdouble ambiente, new_fatt;
        ambiente = luzAmb.Ia * luzAmb.ka;
        new_fatt = 1.0 / pow(closest_intersection, 2);

        /// VETOR SUPERFICIE (S) = lookfrom + distance * direção_normalizada
        GLdouble* superficie = add_arrays(lookfrom, cnt_product(direcao_normal, closest_intersection, 3), 3);

        /// VETOR NORMAL (N) = S - Esfera
        GLdouble* normal = normalizarVetor(sub_arrays(superficie, esferas[sphr_i].coords, 3), 3);

        /// VETOR OBSERVADOR (O) = lookfrom - S
        GLdouble* observador = normalizarVetor(sub_arrays(lookfrom, superficie, 3), 3);

        GLdouble I = ambiente;
        GLdouble ilumAcumuladaLuzPont = 0.0;

        for (int i = 0; i < n_luzes; i++){

            /// VETOR LUZ (L) = Luz - S
            GLdouble* luz = normalizarVetor(sub_arrays(luzPont[i].coords, superficie, 3), 3);

            /// VETOR REFLETIDO (R) = 2N(N.L) - L
            GLdouble* refletido = sub_arrays(
                                    cnt_product(
                                    cnt_product(normal,
                                    dot_product(normal, luz, 3), 3), 2, 3), luz, 3);

            /// CÁLCULO DA ILUMINAÇÃO TOTAL
            GLdouble LN, RO, especular = 0, difusa;

            LN = dot_product(luz, normal, 3);
            RO = dot_product(observador, refletido, 3);

            difusa = esferas[sphr_i].kd * LN;
            if(RO > 0){
                especular = esferas[sphr_i].ks * pow(RO, esferas[sphr_i].nshiny);
            }

            I += luzPont[i].fatt * luzPont[i].IL * (difusa + especular);

            // if(ilumAcumuladaLuzPont < luzPont[i].IL){
            //     ilumAcumuladaLuzPont = luzPont[i].IL;
            // }
            // ilumAcumuladaLuzPont += luzPont[i].IL;
        }

        /// REDUZIR PARA VALOR [0, 1]
        I = I / (2000);

        return I;
    }
    else {
        return -1.0;
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
    //display(n_esferas, n_luzPont);
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
            lumi_xy = calcularIluminacao(i, j, lf, n_esferas, n_luzPont);
            
            if(lumi_xy > 0){
                glBegin(GL_POINTS);
                    glColor3f(lumi_xy, lumi_xy, lumi_xy);   /* Ajusta cor do pixel em escala cinza */
                    glVertex2d(x_tela, y_tela);             /* Desenha um ponto na cor definida em (x_tela, y_tela)*/
                glEnd();

                lumi_xy = -1.0;                             /* Reseta luminancia para -1.0*/
            }
        }
        // somente liberamos o buffer quando TODOS os pixeis forem calculados.
        glFlush ();
    }


    printf("Terminou display\n");
}


struct Esfera constructSphere(GLdouble x, GLdouble y, GLdouble z, GLdouble r, GLdouble kd, GLdouble ks, GLdouble nshiny, GLfloat color_RED, GLfloat color_GREEN, GLfloat color_BLUE){
    GLdouble* coords_esf = (GLdouble*) malloc(sizeof(GLdouble) * 3);
    coords_esf[0] = x;
    coords_esf[1] = y;
    coords_esf[2] = z;
	
	GLdouble* cor_esfera = (GLdouble*) malloc(sizeof(GLdouble) * 3);
	cor_esfera[0] = color_RED;
	cor_esfera[1] = color_GREEN;
	cor_esfera[2] = color_BLUE;
	
    struct Esfera e = {coords_esf, cor_esfera, r, kd, ks, nshiny};

    return e;
}

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
    /// INICIALIZANDO ESFERAS
    // Variáveis de configuração

    n_esferas = 3;

    esferas = (struct Esfera*) malloc(sizeof(*esferas) * n_esferas);

    esferas[0] = constructSphere(0.0, 10.0, -60.0, 3.0, 1.0, 1.0, 30, 255.0, 255.0, 255.0);
    esferas[1] = constructSphere(-10.0, -5.0, -50.0, 8.0, 1.0, 1.0, 30, 255.0, 255.0, 255.0);
    esferas[2] = constructSphere(10.0, 5.0, -40.0, 5.0, 1.0, 1.0, 30,  255.0, 255.0, 255.0);



    /// INICIALIZANDO LUZ AMBIENTE
    // Variáveis de configuração
    luzAmb.Ia = 300;
    luzAmb.ka = 0.8;


    /// INICIALIZANDO LUZES PONTUAIS
    // Variáveis de configuração

    n_luzPont = 3;

    luzPont = (struct LuzPontual*) malloc(sizeof(*luzPont) * n_luzPont);

    luzPont[0] = constructLight(-20.0, -10.0, -10.0, 1000, 1);
    luzPont[1] = constructLight(20.0, -20.0, -10.0, 600, 1);
    luzPont[2] = constructLight(0.0, 40.0, -10.0, 300, 1);


    /// INICIALIZANDO LOOKFROM
    // Variáveis de configuração
    lf = (GLdouble*) malloc(sizeof(GLdouble) * 3);
    lf[0] = 0.0;
    lf[1] = 0.0;
    lf[2] = 10.0;

    /// DEFININDO TAMANHO DO PIXEL DE DESENHO
    glPointSize(1);
}

///***********************///
/// MAIN
///***********************///
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

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize (500, 500);
    glutInitWindowPosition (100, 100);
    glutCreateWindow (argv[0]);
    init ();
    glutDisplayFunc(display);
    //glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
