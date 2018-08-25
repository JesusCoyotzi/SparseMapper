unsigned int u,k;
changer.assembledFloat=space[j].x;
for (u=0,k=0; k < 4; k++,u++)
{
  tmp[u]==changer.byteStream[k]
}
changer.assembledFloat=space[j].y;
for (k=0; k < 4; k++,u++)
{
  tmp[u]==changer.byteStream[k]
}
changer.assembledFloat=space[j].z;
for (k=0; k < 4; k++,u++)
{
  tmp[u]==changer.byteStream[k]
}
changer.assembledInt = colors[partition[j]];
for (k = 0; k < 4; k++, u++) {
        tmp[u]=changer.byteStream[k];
}

if (codebook[partition[j]].z<=freeThr)
{
  for (int v = 0; v < 16; v++) {
    /* code */
    byteBlobFree[i+v]=tmp[v];
  }
  i+=freeCloud.point_step;
}
else
{
  for (int v = 0; v < 16; v++) {
    /* code */
    byteBlobocc[w+v]=tmp[v];
  }
  w+=occCloud.point_step;
}
