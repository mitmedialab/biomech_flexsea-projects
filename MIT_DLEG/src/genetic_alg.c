#include "genetic_alg.h"
int SR_MAX_VALUE = 100;
int SR_MIN_VALUE = -100;
int SR_UPPER_Y = 10000;
int SR_LOWER_Y = -10000;
int SR_FITNESS_CALC_ITER = 3;

/*
Constructor.
Initializes Chrom with int value and int fit.
*/
void Chrom__init(Chrom* self, int value, int fit) {
  self->value = value;
  self->fit = fit;
}

Chrom* Chrom__create(int value, int fit) {
   Chrom* result = (Chrom*)malloc(sizeof(Chrom));
   Chrom__init(result, value, fit);
   return result;
}

//TODO: consider how to deal with decimals

//main function
int run_genetic_calculation()  {
    printf("\nProgram Algorithm starting...\n");
    srand (time(NULL) );

    int num;  // num is the no. of iterations
    current_pop = malloc(6 * sizeof(Chrom*)); //current_pop is array of current generaetion of (6) Chroms

    printf("\nPlease enter the no. of iterations:  ");
    scanf("%d",&num); // enter the number of iterations for genetic program to run

    printf("\n%s", "Initializing values\n");
    for(int i = 0; i <  6; i++) { //Initliaze each of the Chroms in current pop with random value from domain
        printf("\n%s\n", "Setting random value for each chrom\n");
        current_pop[i] = malloc(sizeof(Chrom*));
        (current_pop[i])->value = (rand()%(SR_MAX_VALUE - SR_MIN_VALUE))-SR_MAX_VALUE; //current domain is set to -100 to 100, can be changed
    }

    int testing = 0; //change to 1 for testing

    if(testing == 1) {
        current_pop[0] = malloc(sizeof(Chrom*));
        current_pop[0]->value = -42;

        current_pop[1] = malloc(sizeof(Chrom*));
        current_pop[1]->value = -50;

        current_pop[2] = malloc(sizeof(Chrom*));
        current_pop[2]->value = 10;

        current_pop[3] = malloc(sizeof(Chrom*));
        current_pop[3]->value = 14;

        current_pop[4] = malloc(sizeof(Chrom*));
        current_pop[4]->value = 45;

        current_pop[5] = malloc(sizeof(Chrom*));
        current_pop[5]->value = -5;
    }

    //END TEST

    printf("\n%s\n", "Initial values\n"); //print initial values of Chroms
    for(int i = 0; i < 6; i++) {
        printf("\n%d\n", (current_pop[i])->value);
    }

    for(int i = 0; i <  num; i++) {  //run for num iterations

        printf("\n%s\n", "New Iteration: \n");
        printf("\n\n");

        printf("\n%s\n", "Fitness Calculation: \n");

        for(int j = 0; j < 6; j++) {
            (current_pop[j])->fit = calculate_fitness((current_pop[j])->value);  //calcualate fitness on each Chrom (primarily for testing)
            printf("\n%d\n", (current_pop[j])->fit);
        }

        printf("\n");

        printf("\n%s\n", "Performing Selection \n");

        selection(current_pop); //top 3 parents

        printf("\n Pre mutation new generation \n");

        for(int j = 0; j < 6; j++) {
            printf("\n%d\n", (current_pop[j])->value);
        }

        int index = rand() % 6;  //pick random Chrom to mutate

        printf("\n%s", "Mutation: \n");

        mutate(index); //mutate selected Chrom

        int averaged_fitness;
        int temp;

        printf("\n Re-calculating fitness values for 6 chromes\n");
        for(int j = 0 ; j < 6; j++ ) {
            for(int k = 0; k < SR_FITNESS_CALC_ITER; k++) { //calculate each fitness value three times for each value
                temp = calculate_fitness((current_pop[j])->value);
                while(temp < SR_LOWER_Y && temp > SR_UPPER_Y) { //calculated fitness value is not in desired range
                    temp = calculate_fitness((current_pop[j])->value);
                }
                //generate noise
                int min_nos = 0;
                int max_nos = 0.05 * temp;
//                temp += temp + (rand()%(max_nos - min_nos))-max_nos; //add noise within 5% of value
                averaged_fitness += temp;  //recalculate fitness values for all Chroms
            }
            (current_pop[j])->fit = averaged_fitness / 3;
        }

        printf("\n%s", "Values after mutation: \n"); //Print values after re-mutating
        for(int j = 0; j < 6; j++) {
            printf("\n%d", (current_pop[j])->value);
        }
        printf("\n%s", "End.");
    }

    printf("\n");
    printf("\n%s", "Final values");
    for(int i = 0; i < 6; i++) {    //print out all fitness calculations of final chromes
        (current_pop[i])->fit = calculate_fitness((current_pop[i])->value);
        printf("\n%d", (current_pop[i])->fit);
    }

}

//end of main

//
/*
Calculate fitness.
Calculates & returns y value of function
int x: represents value of chrom, returns f(value) = fit
f(x) can be set to fitness function
*/

int calculate_fitness(int x) {
    int y;
    // y = x*x*x*x + 3*x*x*x+2*x*x+10; //ex function
    y = x*x + 10; //ex function
    return(y);
}

/*
Comparator function.
Comparisons are made based on the fitness values of the Chroms
*/

int cmpfunc (const void * a, const void * b) {
    Chrom * one = (*(Chrom**)a);
    Chrom * two = (*(Chrom**)b);
    return one->fit - two->fit;
}

/*
Selection.
Sorts the Chroms by fitness values. Picks the min values to create children generation.
*/

void selection() {

   printf("Before sorting the list is: \n");
   for(int i = 0 ; i < 6; i++) {
      printf("%d\n",  (current_pop[i])->value);
   }

    qsort(current_pop, 6, sizeof(6 * sizeof(Chrom*)), cmpfunc);

   printf("\nThis is the sorted array:\n"); //print sorted array
   for(int i = 0 ; i < 6; i++ ) {
      printf("%d\n",  (current_pop[i])->value);
   }

    int min_value_arr[] = {(current_pop[0])->value,  (current_pop[1])->value,  (current_pop[2])->value}; //set minimum values


    printf("\n%s", "Best vals inside of function\n");  //print minimum values
    printf("\n%d", min_value_arr[0]);
    printf(" \n%d", min_value_arr[1]);
    printf("\n%d", min_value_arr[2]);

    //creating children

    printf("\n%s", "Calculated children avgs\n");     //calculate averages with those values

    int ch1 = (min_value_arr[0] + min_value_arr[1]) / 2;
    printf("%d\n",  ch1);
    int ch2 = (min_value_arr[1] + min_value_arr[2]) / 2;
    printf("%d\n",  ch2);
    int ch3 = (min_value_arr[0] + min_value_arr[2]) / 2;
    printf("%d\n",  ch3);

    printf("\n");

/*
    //can also compute weighted mean (with decimals)
    int ch1 = (min_value_arr[0]*1 + min_value_arr[1]*0.6);
    printf("%d\n",  ch1);
    int ch2 = (min_value_arr[1]*0.6 + min_value_arr[2]*0.3);
    printf("%d\n",  ch2);
    int ch3 = (min_value_arr[0]*1 + min_value_arr[2]*0.3);
    printf("%d\n",  ch3);
*/

    printf("\n%s", "Changing and printing curr pop vals\n");  //changing current population to previous min (parents) and newly generated childnre
      current_pop[0]->value = ch1;
      printf("%d\n",  (current_pop[0])->value);

    current_pop[1]->value = ch2;
      printf("%d\n",  (current_pop[1])->value);

    current_pop[2]->value = ch3;
      printf("%d\n",  (current_pop[2])->value);

    current_pop[3]->value = min_value_arr[0];
      printf("%d\n",  (current_pop[3])->value);

    current_pop[4]->value = min_value_arr[1];
      printf("%d\n",  (current_pop[4])->value);

    current_pop[5]->value = min_value_arr[2];
      printf("%d\n",  (current_pop[5])->value);
}

/*
Mutate.
Mutates one value from the current population by changing the value by some num from the normal distribution.
mutation_index: the random index to mutate
*/
void mutate(int mutation_index) {


    int mean;
    int sd_mean;
    printf("\n%s", "mutation index");  //calculate mean of all values in current pop
    printf("%d", mutation_index);
    for(int i = 0; i < 6; i++) {
        mean += (current_pop[i])->value;
    }

    mean = mean / 6;
    printf("\n%s", "This is mean ");
    printf("%d", mean);

    for(int i = 0; i < 6; i++) {  //calculate standard deviation
        sd_mean += ((current_pop[i])->value - mean) * ((current_pop[i])->value - mean);
    }

    printf("\n%s", "This is sd_mean ");
    printf("%d", sd_mean/6);

    int standard_deviation = sqrt((sd_mean / 6));

    printf("\n%s", "This is stand dev "); //print standard deviation
    printf("%d", standard_deviation);

    int mutated_value;

   /*  int bin = rand() % 2;

    if (bin==0) {
        printf("\n%s", "Plus standard deviation");
        mutated_value = mutation + standard_deviation;
    }

    else if (bin==1) {
        printf("\n%s", "Minus standard deviation");
        mutated_value = mutation - standard_deviation;
    }
    else {
        printf("\n%s", "This is bin: ");
        printf("%d", bin);
    } */

    printf("\n%s", "pre mutated value: ");
    printf("%d", (current_pop[mutation_index]->value));

    int operation = rand() % 6; //pick random operation
    printf("\n%s", "Operation: ");
    printf("%d", operation);
    switch(operation) {
        case 1:
            printf("\nCase 1");
            mutated_value = (current_pop[mutation_index]->value) + 1 * standard_deviation;
            break;
        case 2:
            printf("\nCase 1");
            mutated_value = (current_pop[mutation_index]->value) + 2 * standard_deviation;
            break;
        case 3:
            mutated_value = (current_pop[mutation_index]->value) + 3 * standard_deviation;
            break;
        case 4:
            mutated_value = (current_pop[mutation_index]->value) - 1 * standard_deviation;
            break;
        case 5:
            mutated_value = (current_pop[mutation_index]->value) - 2 * standard_deviation;
            break;
        case 6:
            mutated_value = (current_pop[mutation_index]->value) - 3 * standard_deviation;
            break;
        default:
            mutated_value = (current_pop[mutation_index]->value);
            break;
    }



    printf("\n%s", "Mutation occurred. Replacing Value. Value to mutate: ");
    printf("%d", (current_pop[mutation_index]->value));

    (current_pop[mutation_index]->value) = mutated_value;  //substitute mutated value
    printf("\n%s", "mutated value ");
    printf("%d", mutated_value);
}


/*int *next_generation(Chrom* top_three[]) {
    int ch1 = (top_three[0]->value + top_three[1]->value) / 2;
    int ch2 = (top_three[1]->value + top_three[2]->value) / 2;
    int ch3 = (top_three[2]->value + top_three[2]->value) / 2;


    //pointer or on stack?
    Chrom* next_gen[] = {ch1, ch2, ch3, top_three[0]->value, top_three[1]->value, top_three[2]->value};


    for(int i = 0; i < 6; i++) {
        printf("\n%d", next_gen[i]->value);
    }

    return next_gen;
} */
