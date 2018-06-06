/* 
 * File:   main.c
 * Author: pehladik
 *
 * Created on 23 décembre 2017, 19:45
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "./header/functions.h"

// Déclaration des taches
RT_TASK th_server;
RT_TASK th_sendToMon;
RT_TASK th_receiveFromMon;
RT_TASK th_openComRobot;
RT_TASK th_startRobot;
RT_TASK th_move;
RT_TASK th_battery;
RT_TASK th_startCamera;
RT_TASK th_pictures;
RT_TASK th_cleaner;

// Déclaration des priorités des taches
int PRIORITY_TSERVER = 30;
int PRIORITY_TOPENCOMROBOT = 20;
int PRIORITY_TMOVE = 20;
int PRIORITY_TBATTERY = 10;
int PRIORITY_TSENDTOMON = 25;
int PRIORITY_TRECEIVEFROMMON = 22;
int PRIORITY_TSTARTROBOT = 20;
int PRIORITY_TSTARTCAMERA = 20;
int PRIORITY_TPICTURES = 15;
int PRIORITY_TCLEANER = 30;

RT_MUTEX mutex_robotStarted;
RT_MUTEX mutex_move;
RT_MUTEX mutex_comFails;
RT_MUTEX mutex_cameraStarted;
RT_MUTEX mutex_camera;
RT_MUTEX mutex_locate;
RT_MUTEX mutex_arene;

// Déclaration des sémaphores
RT_SEM sem_barrier;
RT_SEM sem_openComRobot;
RT_SEM sem_serverOk;
RT_SEM sem_startRobot;
RT_SEM sem_startCamera;
RT_SEM sem_probeArena;
RT_SEM sem_serverKo;
RT_SEM sem_reset;

// Déclaration des files de message
RT_QUEUE q_messageToMon;

int MSG_QUEUE_SIZE = 10;

// Déclaration des ressources partagées
int etatCommMoniteur = 1;
int robotStarted = 0;
int cameraStarted = 0;
int comFails = 0;
int locate = 0;
char move = DMB_STOP_MOVE;
Arene arene;
int areneConfirmee = 0;
int confirmationArene = 0;
Camera cam;

/**
 * \fn void initStruct(void)
 * \brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void initStruct(void);

/**
 * \fn void startTasks(void)
 * \brief Démarrage des tâches
 */
void startTasks(void);

/**
 * \fn void deleteTasks(void)
 * \brief Arrêt des tâches
 */
void deleteTasks(void);

int main(int argc, char **argv) {
    int err;
    //Lock the memory to avoid memory swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("#################################\n");
    printf("#      DE STIJL PROJECT         #\n");
    printf("#################################\n");

    while(1)
    {
        initStruct();
        startTasks();
        rt_sem_broadcast(&sem_barrier);

        rt_sem_p(&sem_reset, TM_INFINITE);

        deleteTasks();
    }

    return 0;
}

void initStruct(void) {
    int err;
    /* Creation des mutex */
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cameraStarted, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_comFails, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_locate, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arene, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    /* Creation du semaphore */
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startCamera, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_probeArena, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverKo, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reset, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    /* Creation des taches */
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startCamera, "th_startCamera", 0, PRIORITY_TSTARTCAMERA, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_pictures, "th_pictures", 0, PRIORITY_TPICTURES, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cleaner, "th_cleaner", 0, PRIORITY_TCLEANER, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    /* Creation des files de messages */
    if (err = rt_queue_create(&q_messageToMon, "toto", MSG_QUEUE_SIZE * sizeof (MessageToRobot), MSG_QUEUE_SIZE, Q_FIFO)) {
        printf("Error msg queue create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    // Réinit. des variables globales   
    etatCommMoniteur = 1;
    robotStarted = 0;
    cameraStarted = 0;
    comFails = 0;
    locate = 0;
    move = DMB_STOP_MOVE;
    areneConfirmee = 0;
    confirmationArene = 0;    
}

void startTasks() {

    int err;
    
    if (err = rt_task_start(&th_startRobot, &f_startRobot, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startCamera, &f_startCamera, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, &f_receiveFromMon, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, &f_sendToMon, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, &f_openComRobot, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, &f_move, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, &f_battery, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_pictures, &f_pictures, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cleaner, &f_cleaner, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    // Démarrer à la fin
    if (err = rt_task_start(&th_server, &f_server, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
}

void deleteTasks() {
    rt_task_delete(&th_server);
    rt_task_delete(&th_openComRobot);
    rt_task_delete(&th_move);
    rt_task_delete(&th_battery);
    rt_task_delete(&th_pictures);
    rt_task_delete(&th_cleaner);
    rt_task_delete(&th_sendToMon);
    rt_task_delete(&th_receiveFromMon);
    rt_task_delete(&th_startCamera);
    rt_task_delete(&th_startRobot);
    
    rt_queue_delete(&q_messageToMon);
}
