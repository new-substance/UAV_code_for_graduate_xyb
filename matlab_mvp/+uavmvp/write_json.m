function write_json(path,value)
f=fopen(path,'w','n','UTF-8');assert(f>=0,'uavmvp:outputOpen','Cannot create output.');
closer=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true));
end
