#include <kilolib.h>
#include <stdbool.h>
#define DEBUG
#include <debug.h>
// Only needed for debugging
#include <inttypes.h>
// Constants for orbit control.


// Constants for motion handling function.
#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

int current_motion = STOP;
int TOO_CLOSE_DISTANCE = 0;
int DESIRED_DISTANCE = 0;
uint16_t current_orbit = 0;
uint16_t temp_orbit = 0;
int distance = 0;

// Define the number of neighbors we accept
#define maxN 4

#define TTL_LENGTH 4

// Declare a structure to represent each neighbor
typedef struct {
    uint16_t id;
    int distance;
    int gradient;
    uint8_t reading_ttl;
    int16_t reading;
    uint32_t timestamp;
} neighbor_t;

// Message flags
int new_message = 0;
int message_sent = 0;

// dummy values for light reading functionality
int16_t my_reading;
uint8_t id_a, id_b;
uint8_t ttl;
uint16_t id_full;
bool mine = true;
int own_gradient = 0;
int received_gradient = 0;
// Message to transmit
message_t message;

// Create list of neighbors
neighbor_t neighbors[maxN];

// Track the number of nearest neighbors
int N_neighbors = 0;

// Function to sample light
int16_t sample_light() {
    
    int numsamples = 0;
    int sum = 0;
    while (numsamples < 50) {
        int sample = get_ambientlight();
        if (sample != -1) {
            sum = sum + sample;
            numsamples++;
        }
    }
    return (sum / 50);
}
// Function to handle motion.
void set_motion(int new_motion)
{
    // Only take an action if the motion is being changed.
    if (current_motion != new_motion)
    {
        current_motion = new_motion;
        
        if (current_motion == STOP)
        {
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == LEFT)
        {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == RIGHT)
        {
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

int max_reading(){
    int index = -1;
    int max = my_reading;
    for (int i = 0; i < N_neighbors; i++){
        if ((neighbors[i].reading > max) )//&& (neighbors[i].reading_ttl > 0))
        {
            max = neighbors[i].reading;
            index = i;
        }
    }
    
    return index;
}



void setup() {
    // Initialize message:
    // The type is always NORMAL.
    message.type = NORMAL;
    // Seed the software random number generator using
    // a hardware random number generator
    rand_seed(rand_hard());
    // Give the kilobot a unique ID
    id_a = rand_soft();
    id_b = rand_soft();
    id_full = ((uint16_t)id_a << 8) | id_b;
    my_reading = sample_light();
    
    
    // Load data
    message.data[0] = id_a;
    message.data[1] = id_b;
    message.data[2] = my_reading >> 8;
    message.data[3] = 0 | my_reading;
    message.data[4] = own_gradient;
    // It's important that the CRC is computed after the data has been set;
    // otherwise it would be wrong and the message would be dropped by the
    // receiver.
    message.crc = message_crc(&message);
}

// Removes neighbors who are no longer in contact
void purge(void){
    int8_t i;
    for (i = N_neighbors-1; i >= 0; i--){
        
        if (kilo_ticks - neighbors[i].timestamp  > 64) //32 ticks = 1 s
        {
            //this one is too old.
            neighbors[i] = neighbors[N_neighbors-1];
            //replace it by the last entry
            N_neighbors--;
        }
    }
}

void loop() {
    if (message_sent == 1){
        if (max_reading() == -1){
            // My reading is the highest STATE 1
            own_gradient = 0;
            set_color(RGB(1, 1, 1)); // White
            set_motion(STOP);
        }
        else {
            // My light reading is not the highest STATE 2
            int lowest_grad = 255;
            
            for (int i = 0; i < N_neighbors; i++){
                if (neighbors[i].gradient < lowest_grad)
                {
                    lowest_grad = neighbors[i].gradient;
                    distance = neighbors[i].distance;
                    temp_orbit = neighbors[i].id;
                }
            }
            own_gradient = lowest_grad + 1;
            /*
            //MOTION
            if (temp_orbit != current_orbit){
                DESIRED_DISTANCE = distance;
                TOO_CLOSE_DISTANCE = distance - 15;
                current_orbit = temp_orbit;
            }
            // If too close, move forward to get back into orbit.
            if (distance < TOO_CLOSE_DISTANCE)
            {
                set_color(RGB(0, 1, 0));
                set_motion(FORWARD);
            }
            // If not too close, turn left or right depending on distance,
            // to maintain orbit.
            else
            {
                if (distance < DESIRED_DISTANCE)
                {
                    set_color(RGB(1, 0, 0));
                    set_motion(LEFT);
                }
                else
                {
                    set_color(RGB(0, 0, 1));
                    set_motion(RIGHT);
                }
            }*/
        }
              
        printf("Gradient Value %i \n", own_gradient);
        printf("Max Value %i \n", ((int16_t)message.data[2] << 8) | message.data[3]);
        printf("ID %" PRId16 "\n", id_full);
        printf("My light value %" PRId16 "\n", my_reading);
        //printf("Orbit ID %" PRId16 "\n", current_orbit);
        //printf("Distance to Orbit %i\n", DESIRED_DISTANCE);
        printf("////////\n");
        
        //Show your colors!
        if (own_gradient == 1){
            set_color(RGB(1,0,0));
        }
        else if (own_gradient == 2){
            set_color(RGB(0,1,0));
        }
        else if (own_gradient == 3){
            set_color(RGB(0,0,1));
        }
        else if (own_gradient >3){
            set_color(RGB(0,1,1));
        }
        
        
        
        message_sent = 0;
    }
    
}


message_t *message_tx()
{
    
    if (own_gradient > 0){
        my_reading = sample_light();
    }
    int max_member = max_reading();
    
    // Change the Message Data
    if (max_member != -1){
        message.data[2] = neighbors[max_member].reading >> 8;
        message.data[3] = 0 |  neighbors[max_member].reading;
        message.data[4] = own_gradient;
        //neighbors[max_member].reading_ttl--;
        //message.data[5] = neighbors[max_member].reading_ttl;
    }
    else {
        message.data[2] = my_reading >> 8;
        message.data[3] = 0 | my_reading;
        message.data[4] = own_gradient;
        //message.data[5] = TTL_LENGTH;
    }
    
    // Recompute the checksum
    message.crc = message_crc(&message);
    return &message;
    
}

void message_tx_success()
{
    //printf("id %i max %i \n", kilo_uid, max_reading);
    // Set the flag on message transmission.
    message_sent = 1;
    // Forget old neigbors
    purge();
}



void message_rx(message_t *m, distance_measurement_t *distance)
{
    int i = 0;
    // Set the flag on message reception.
    new_message = 1;
    uint16_t id = ((uint16_t)m->data[0] << 8) | m->data[1];
    int16_t read = ((int16_t)m->data[2] << 8) | m->data[3];
    // Check to see if the id is in the list of neighbors
    for (i = 0; i < N_neighbors; i++){
        //printf("Neighbor Count %i id %i time %" PRId32 "\n",N_neighbors, neighbors[i].id,neighbors[i].timestamp);
        
        if (neighbors[i].id == id)
        {// found it
            neighbors[i].distance = estimate_distance(distance);
            neighbors[i].timestamp = kilo_ticks;
            neighbors[i].reading = read;
            neighbors[i].gradient = m->data[4];
            //neighbors[i].reading_ttl = m->data[5];
            break;
        }
    }
    //if (N_neighbors < maxN-1) // if we have too many neighbors,
    if (i == N_neighbors){
        N_neighbors++;
        // i now points to where this message should be stored
        neighbors[i].id = id;
        neighbors[i].distance = estimate_distance(distance);
        neighbors[i].timestamp = kilo_ticks;
        neighbors[i].reading = read;
        neighbors[i].gradient = m->data[4];
        //neighbors[i].reading_ttl = m->data[5];
    }
    new_message = 0;
}



int main() {
    debug_init();
    // initialize hardware
    kilo_init();
    // Register the message_tx callback function
    kilo_message_tx = message_tx;
    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;
    // Register the message_rx callback function.
    kilo_message_rx = message_rx;
    // start program
    kilo_start(setup, loop);
    
    return 0;
}
