#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

FILE   *fp = NULL;
float  delta;

void main (int argc, char **argv)
{
	float    delta;
	float 	 minVal;
	float 	 maxVal;
	float    tblSize;
	float    step;
	float    value;
	uint16_t i;
	
	if (argc != 5)
	{
	    printf("argc   = %d\n", argc);
		printf("Usage: table[.exe] <minVal> <maxVal> <tblSize> <name>\n");
		printf("       <minVal>  : Untergrenze\n");
		printf("       <maxVal>  : Obergrenze\n");
		printf("       <tblSize> : Groesse der Tabelle\n");	
		printf("       <name>    : Name der Zieldatei\n");
		return;
	}

	minVal  = atof(argv[1]);
	maxVal  = atof(argv[2]);
	tblSize = atof(argv[3]);
	
	delta = (maxVal - minVal)/tblSize;
	step  = 3300.0 / tblSize;
	
	printf("minVal = %f\n", minVal);
	printf("maxVal = %f\n", maxVal);
	printf("maxVal = %f\n", tblSize);
	printf("delta  = %f\n", delta);
	printf("name   = %s\n", argv[4]);
	

	if ((fp = fopen(argv[4], "w+")) == NULL)
	{
		printf("Kann Datei %sTable nicht oeffnen.\n");
		return;
	}
	
	fprintf(fp, "const uint16_t %sTable[] = \n", argv[4]);
	fprintf(fp, "{\n");
	
	

	value = minVal;
	for (i = 0; i < tblSize; i++)
	{
		fprintf(fp, "\t%4d, ", (uint16_t) (value/delta));	

		if ((i > 0) && (i % 10 == 0))
		{
			fprintf(fp, "\n");
		}
		
		if (i % 50 == 0)
		{
			fprintf(fp, "\n");
		}
		
		value += delta;
	}
	fprintf(fp, "\n};\n");
	fclose(fp);
}