import numpy
import sys
import os

from mpi4py import MPI
comm = MPI.COMM_WORLD
rank = comm.Get_rank()

# randNum = numpy.zeros(1)

# if rank == 1:
#         randNum = numpy.random.random_sample(1)
#         print("Process", rank, "drew the number", randNum[0])
#         comm.Send(randNum, dest=0, tag=0)
#         comm.Recv(randNum, source=0, tag=1)
#         print("Process", rank, "received the number", randNum[0])
        
# if rank == 0:
#         print("Process", rank, "before receiving has the number", randNum[0])
#         comm.Recv(randNum, source=1, tag=0)
#         print("Process", rank, "received the number", randNum[0])
#         randNum *= 2
#         comm.Send(randNum, dest=1, tag=1)

# randNum = numpy.zeros(1)
# diffNum = numpy.random.random_sample(1)

# if rank == 1:
#         randNum = numpy.random.random_sample(1)
#         print("Process", rank, "drew the number", randNum[0])
#         comm.Isend(randNum, dest=0)
#         req = comm.Irecv(randNum, source=0)
#         req.Wait()
#         print("Process", rank, "received the number", randNum[0])
        
# if rank == 0:
#         print("Process", rank, "before receiving has the number", randNum[0])
#         req = comm.Irecv(randNum, source=1)
#         #req.Wait()
#         print("Process", rank, "received the number", randNum[0])
#         randNum *= 2
#         comm.Isend(randNum, dest=1)

arguments = sys.argv
#agents = str(arguments[1][1])
print(arguments)