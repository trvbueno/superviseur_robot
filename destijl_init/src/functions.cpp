#include "../header/functions.h"

char mode_start;

void write_in_queue(RT_QUEUE *, MessageToMon);

void f_cleaner(void *arg) {
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
#ifdef _WITH_TRACE_
        printf("%s: ready to reset supervisor\n", info.name);
#endif
        
    rt_sem_p(&sem_serverKo, TM_INFINITE);
    
    printf("%s: /!\\ monitor is unreachable, resetting supervisor /!\\\n", info.name);
        
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted = 0;
    rt_mutex_release(&mutex_robotStarted);
    
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    cameraStarted = 0;
    rt_mutex_release(&mutex_robotStarted);
    
    close_camera(&cam);
    close_communication_robot();
    
    rt_sem_v(&sem_reset);
}

void f_server(void *arg) {
    int err;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    err = run_nodejs("/usr/local/bin/node", "/home/pi/Interface_Robot/server.js");

    if (err < 0) {
        printf("Failed to start nodejs: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } else {
#ifdef _WITH_TRACE_
        printf("%s: nodejs started\n", info.name);
#endif
        open_server();
        rt_sem_broadcast(&sem_serverOk);
    }
}

void f_pictures(void * arg) {
    int ret = 0;
    Image photo;
    Jpg photo_compr;
    
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif

    /* Période de 100 ms */
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
#ifdef _WITH_TRACE_
        //printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        // printf("%s: Periodic activation\n", info.name);
#endif
        // Boucle de traitement
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        if (cameraStarted == 1)
        {            
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            get_image(&cam, &photo);
            rt_mutex_release(&mutex_camera);
            
            Arene nouvelleArene;
                                
            /* 1ms pour attendre probeArena */
            if(rt_sem_p(&sem_probeArena, 1000000) == 0)
            {
                // Demande de localisation de l'arène
                if(detect_arena(&photo, &nouvelleArene) == 0)
                {
                    // On a trouvé une arène
                    draw_arena(&photo, &photo, &nouvelleArene);
                    compress_image(&photo, &photo_compr);
                    send_message_to_monitor(HEADER_STM_IMAGE, &photo_compr); // hack :(
                    
                    // On attend une confirmation/infirmation du moniteur
                    rt_sem_p(&sem_probeArena, TM_INFINITE);
                    rt_mutex_acquire(&mutex_arene, TM_INFINITE);
                    if(confirmationArene == 1)
                    {
                        // L'arène est confirmée
                        areneConfirmee = 1;
                        confirmationArene = 0;
                        arene = nouvelleArene;
                    }
                    rt_mutex_release(&mutex_arene);
                }
                else
                {
                    // On a pas trouvé d'arène
                    MessageToMon msg;
                    set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
                    write_in_queue(&q_messageToMon, msg);
                }
            }
            
            rt_mutex_acquire(&mutex_arene, TM_INFINITE);
            if (areneConfirmee == 1)
            {
                rt_mutex_acquire(&mutex_locate, TM_INFINITE);
                if (locate == 1)
                {
                    Position pos_robot;
                    int ret = 0;

                    if (ret = detect_position(&photo, &pos_robot, &arene) > 0)
                    {
                        draw_position(&photo, &photo, &pos_robot);

                        MessageToMon msg;
                        set_msgToMon_header(&msg, HEADER_STM_POS);
                        set_msgToMon_data(&msg, &pos_robot);
                        write_in_queue(&q_messageToMon, msg);                        
                    }
                }
                rt_mutex_release(&mutex_locate);
                
                // On dessine l'arène
                draw_arena(&photo, &photo, &arene);
            }
            rt_mutex_release(&mutex_arene);
            
            compress_image(&photo, &photo_compr);
            //MessageToMon msg;
            //set_msgToMon_header(&msg, HEADER_STM_IMAGE);
            //set_msgToMon_data(&msg, &photo_compr);
            //write_in_queue(&q_messageToMon, msg); ne fonctionne pas
            send_message_to_monitor(HEADER_STM_IMAGE, &photo_compr); // hack :(
        }
        rt_mutex_release(&mutex_cameraStarted);
    }
}

void f_sendToMon(void * arg) {
    int err;
    MessageToMon msg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {

#ifdef _WITH_TRACE_
        printf("%s : waiting for a message in queue\n", info.name);
#endif
        if (rt_queue_read(&q_messageToMon, &msg, sizeof (MessageToRobot), TM_INFINITE) >= 0) {
#ifdef _WITH_TRACE_
            printf("%s : message {%s,%s} in queue\n", info.name, msg.header, msg.data);
#endif
            err = send_message_to_monitor(msg.header, msg.data);
            free_msgToMon_data(&msg);
            rt_queue_free(&q_messageToMon, &msg);
            
            // Send error
            if(err == -1)
            {
#ifdef _WITH_TRACE_
                printf("%s : error sending message to monitor\n", info.name);
#endif
                rt_sem_v(&sem_serverKo);
            }
        } else {
            printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void f_receiveFromMon(void *arg) {
    MessageFromMon msg;
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    do {
#ifdef _WITH_TRACE_
        printf("%s : waiting for a message from monitor\n", info.name);
#endif
        err = receive_message_from_monitor(msg.header, msg.data);
        printf("%d\n", err);
        
        // Receive error
        if(err <= 0)
        {
#ifdef _WITH_TRACE_
            printf("%s : error receiving message from monitor\n", info.name);
#endif
            rt_sem_v(&sem_serverKo);
            pause();
        }
        
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
            if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_openComRobot);
            }
        } else if (strcmp(msg.header, HEADER_MTS_CAMERA) == 0) {
            if (msg.data[0] == CAM_OPEN) { // Open camera
#ifdef _WITH_TRACE_
                printf("%s: message open camera\n", info.name);
#endif
                rt_sem_v(&sem_startCamera);
            }
            else if (msg.data[0] == CAM_CLOSE) { // Open camera
#ifdef _WITH_TRACE_
                printf("%s: message close camera\n", info.name);
#endif
                rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
                cameraStarted = 0;
                rt_mutex_release(&mutex_cameraStarted);
            } 
            else if (msg.data[0] == CAM_COMPUTE_POSITION) {
                rt_mutex_acquire(&mutex_locate, TM_INFINITE);
                locate = 1;
                rt_mutex_release(&mutex_locate);
            } 
            else if (msg.data[0] == CAM_STOP_COMPUTE_POSITION) {
                rt_mutex_acquire(&mutex_locate, TM_INFINITE);
                locate = 0;
                rt_mutex_release(&mutex_locate);
            }
            else if (msg.data[0] == CAM_ASK_ARENA) {
                rt_sem_v(&sem_probeArena);
            }
            else if (msg.data[0] == CAM_ARENA_CONFIRM) {
                rt_mutex_acquire(&mutex_arene, TM_INFINITE);
                confirmationArene = 1;
                rt_mutex_release(&mutex_arene);
                rt_sem_v(&sem_probeArena);
            }
            else if (msg.data[0] == CAM_ARENA_INFIRM) {
                rt_mutex_acquire(&mutex_arene, TM_INFINITE);
                confirmationArene = -1;
                rt_mutex_release(&mutex_arene);
                rt_sem_v(&sem_probeArena);
            }
        } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
            if (msg.data[0] == DMB_START_WITHOUT_WD) { // Start robot
#ifdef _WITH_TRACE_
                printf("%s: message start robot\n", info.name);
#endif 
                rt_sem_v(&sem_startRobot);

            } else if ((msg.data[0] == DMB_GO_BACK)
                    || (msg.data[0] == DMB_GO_FORWARD)
                    || (msg.data[0] == DMB_GO_LEFT)
                    || (msg.data[0] == DMB_GO_RIGHT)
                    || (msg.data[0] == DMB_STOP_MOVE)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msg.data[0];
                rt_mutex_release(&mutex_move);
#ifdef _WITH_TRACE_
                printf("%s: message update movement with %c\n", info.name, move);
#endif

            }
        } 
    } while (err > 0);

}

void f_openComRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
        err = open_communication_robot();
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_startCamera(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startCamera\n", info.name);
#endif
        rt_sem_p(&sem_startCamera, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startCamera arrived => starting camera\n", info.name);
#endif
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        err = open_camera(&cam);
        rt_mutex_release(&mutex_camera);
        
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the camera is opened\n", info.name);
#endif
            rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
            cameraStarted = 1;
            rt_mutex_release(&mutex_cameraStarted);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);                                   
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_startRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRobot\n", info.name);
#endif
        rt_sem_p(&sem_startRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startRobot arrived => Start robot\n", info.name);
#endif
        err = send_command_to_robot(DMB_START_WITHOUT_WD);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_comFails, TM_INFINITE);
            comFails = 0;
            rt_mutex_release(&mutex_comFails);
            
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
                        
            /* On vérifie maintenant que la communication avec lui est maintenue*/
            while(1)
            {
                /* On calme la tache de 100 ms */
                rt_task_sleep(100000000);
        
                rt_mutex_acquire(&mutex_comFails, TM_INFINITE);
                if(comFails > 3) /* normalement 3 */
                {
                    rt_mutex_release(&mutex_comFails);
#ifdef _WITH_TRACE_
                    printf("%s communication loss with robot, closing and reseting\n", info.name);
#endif
                    /* On indique aux autres threads que le robot est arreté */
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted = 0;
                    rt_mutex_release(&mutex_robotStarted);
                    
#ifdef _WITH_TRACE_
                    printf("signaled to other threads that robot is off\n", info.name);
#endif
                    /* On clos la communicatoin */
                    close_communication_robot();
                    
#ifdef _WITH_TRACE_
                    printf("closed robot radio port\n", info.name);
#endif
                    
                    /* On notifie le moniteur */
                    MessageToMon msg;
                    set_msgToMon_header(&msg, HEADER_STM_LOST_DMB);
                    write_in_queue(&q_messageToMon, msg);
                    
                    break; /* On sort de la boucle secondaire pour se remettre en attente d'une demande de connexion */
                }                
                rt_mutex_release(&mutex_comFails);
            }
            
#ifdef _WITH_TRACE_
             printf("robot shut off, preparing to reboot\n", info.name);
#endif
           
        } else {
#ifdef _WITH_TRACE_
            printf("robot did not want to start\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_battery(void *arg) {
    int ret = 0;
    
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif

    /* Période de 500 ms */
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
#ifdef _WITH_TRACE_
        //printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        //printf("%s: Periodic activation\n", info.name);
        //printf("%s: move equals %c\n", info.name, move);
#endif

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted == 1) {
            ret = send_command_to_robot(DMB_GET_VBAT);
#ifdef _WITH_TRACE_
            //printf("%s: the battery status command %c was sent\n", info.name, DMB_GET_VBAT);
#endif
            if(ret == ROBOT_ERROR || ret == ROBOT_TIMED_OUT)
            {
#ifdef _WITH_TRACE_
                printf("%s: communication error while checking battery status %c\n", info.name, ret);
#endif
                rt_mutex_acquire(&mutex_comFails, TM_INFINITE);
                comFails += 1;
                rt_mutex_release(&mutex_comFails);
            }
            else
            {
                ret += 48;
                MessageToMon msg;
                set_msgToMon_header(&msg, HEADER_STM_BAT);
                set_msgToMon_data(&msg, &ret);
                write_in_queue(&q_messageToMon, msg);
            }
        }
                
        rt_mutex_release(&mutex_robotStarted);
    }
}

    void f_move(void *arg) {
        int ret;
        
        /* INIT */
        RT_TASK_INFO info;
        rt_task_inquire(NULL, &info);
        printf("Init %s\n", info.name);
        rt_sem_p(&sem_barrier, TM_INFINITE);

        /* PERIODIC START */
#ifdef _WITH_TRACE_
        printf("%s: start period\n", info.name);
#endif
        rt_task_set_periodic(NULL, TM_NOW, 100000000);
        while (1) {
#ifdef _WITH_TRACE_
            //printf("%s: Wait period \n", info.name);
#endif
            rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
            //printf("%s: Periodic activation\n", info.name);
            //printf("%s: move equals %c\n", info.name, move);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            if (robotStarted) {
                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                ret = send_command_to_robot(move);
                rt_mutex_release(&mutex_move);
                
                if(ret == ROBOT_ERROR || ret == ROBOT_TIMED_OUT)
                {
#ifdef _WITH_TRACE_
                    printf("%s: communication error while send move cmd %c\n", info.name, ret);
#endif
                    rt_mutex_acquire(&mutex_comFails, TM_INFINITE);
                    comFails += 1;
                    rt_mutex_release(&mutex_comFails);
                }
                
#ifdef _WITH_TRACE_
                printf("%s: the movement %c was sent\n", info.name, move);
#endif            
            }
            rt_mutex_release(&mutex_robotStarted);
        }
    }

    void write_in_queue(RT_QUEUE *queue, MessageToMon msg) {
        void *buff;
        buff = rt_queue_alloc(&q_messageToMon, sizeof (MessageToMon));
        memcpy(buff, &msg, sizeof (MessageToMon));
        rt_queue_send(&q_messageToMon, buff, sizeof (MessageToMon), Q_NORMAL);
    }