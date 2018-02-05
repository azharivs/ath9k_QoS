BEGIN{
	avg = 0;
	avg_counter = 0;
	sum = 0;
	i = 0;
	
	MAX = 1000;
}
{
	sum += $1;
	if (++i >= MAX)
	{
		avg += sum/MAX;
		avg_counter++;
		
		sum = 0;
		i = 0;
	}
}
END{

	if (avg_counter == 0)
	{
		if (i != 0)
			avg = sum/i;
	}
	else
	{
		avg = avg + (sum/MAX);
		avg_counter += (i/MAX);
		avg /= avg_counter;
	}
	print avg;
}