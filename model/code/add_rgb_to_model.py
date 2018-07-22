for i in range(10):
	print(i)
	origin_path="./#%d.txt"%(i+1)
	save_path="./#%d_rgb.txt"%(i+1)

	data=[]
	with open(origin_path,"r") as f:
		lines=f.readlines()
		for line in lines:
			line=line.replace(","," ")
			line=line.split("\n")[0]+' 255 255 255\n'
			data.append(line)
	with open(save_path,'w')as f:
		for line in data:
			f.write(line)

