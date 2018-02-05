BEGIN{
	i = 1;
}
{
	j = 0;
	for(j ; j < 10; j++)
		if ($j == "sec")
			break;
	j += 3;
	
	bw = $j;
	unit = $(j+1);
	
	// Converting Kbps to Mbps
	if (unit == "Kbits/sec")
	{
		bw /= 1000;
		unit = "Mbits/sec";
	}
	
	print i "\t" bw, unit;
	i++;
}
END{
}