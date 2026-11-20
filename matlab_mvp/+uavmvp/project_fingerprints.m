function out=project_fingerprints()
%PROJECT_FINGERPRINTS Portable exact-byte identity; never copy source per case.
main=fileparts(fileparts(mfilename('fullpath')));repo=fileparts(main);
out=struct('specification',hash_file(fullfile(repo,'论文总纲.md')), ...
    'configuration',hash_file(fullfile(repo,'research_configs','baseline.json')));
roots={main,fullfile(main,'+uavmvp'),fullfile(main,'+uavstage'), ...
    fullfile(main,'+uavpriority'),fullfile(repo,'research_checks')};
entries={};
for k=1:numel(roots)
    files=dir(fullfile(roots{k},'*.m'));
    for j=1:numel(files)
        path=fullfile(files(j).folder,files(j).name);
        relative=strrep(path(numel(repo)+2:end),'\','/');
        entries{end+1}=[relative,' ',hash_file(path)]; %#ok<AGROW>
    end
end
entries=sort(entries);out.source=hash_bytes(unicode2native(strjoin(entries,newline),'UTF-8'));
out.source_file_count=numel(entries);
end
function out=hash_file(path)
fid=fopen(path,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
out=hash_bytes(fread(fid,Inf,'*uint8'));
end
function out=hash_bytes(bytes)
digest=java.security.MessageDigest.getInstance('SHA-256');
digest.update(typecast(uint8(bytes(:)),'int8'));raw=typecast(digest.digest(),'uint8');
out=lower(reshape(dec2hex(raw,2)',1,[]));
end
