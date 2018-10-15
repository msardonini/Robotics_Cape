

//system Includes
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <stdio.h>
#include <getopt.h>


//Ours
#include "logger.h"


static void print_usage(){
    std::cout <<"\n Usage:\n";
    std::cout <<"./projPoints [-OPTION OPTION_VALUE]\n";
    std::cout <<"\n";
    std::cout <<"Options:\n";
    std::cout <<"-f {file}                  input file to use:\n";
    std::cout <<"-h {help}                  Print this usage text\n";
    
    return;
}




int main(int argc, char *argv[])
{

	FILE *fdInput;
	char* inputfile;

	//Struct to hold in all the input data
	core_log_entry_t *lg;
	lg = (core_log_entry_t*)malloc(sizeof(core_log_entry_t));

	int c;
	while ((c = getopt (argc, argv, "f:h")) != -1)
    {
        switch (c)
        {
            //Get the filepath of the input file
            case 'f':
                inputfile = static_cast<char*>(calloc(256, sizeof(char)));
                if(sscanf(optarg, "%s", inputfile) <= 0)
                {
                    std::cerr << "Failed to read input file" << std::endl;
                    return 0;
                }
                break;
           
            //Print out the help guide
            case 'h':
                print_usage();
                return 0;

            //Handle unknown Arguments
            case '?':
                if (optopt == 'c')
                  std::cerr << "Option -%c requires an argument.\n" << optopt;
                else if (isprint (optopt))
                  std::cerr << "Unknown option `-%c'.\n" << optopt;
                else
                    std::cerr << "Unknown option character " << optopt;
                    print_usage();
                return 1;
            
            //Error on Unknown input Argument
            default:
                std::cout <<"Error! Unknown argument given";
                print_usage();
                return 0;
        }
    }


    //Open the file chosen for replaying
    fdInput = fopen(inputfile, "r");

    //Don't bother reading in the header    
    fscanf(fdInput,"%*[^\n]\n");

    int ret = 0;
    int i;
    while(ret == 0)
    {
    	ret = 0;
		#define X(type, fmt, name) if(fscanf(fdInput, fmt "," , &lg->name) == EOF)  ret = EOF;
		CORE_LOG_TABLE
		#undef X
		fscanf(fdInput,"\n");



    }	
	fclose(fdInput);

}